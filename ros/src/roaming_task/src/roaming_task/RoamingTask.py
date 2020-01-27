#!/usr/bin/env python3

from typing import Tuple, List
import math
import numpy
import numpy as np
from math import inf
from scipy import signal
from random import randrange, shuffle

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from roaming_task.srv import GetRoamingTask, GetRoamingTaskRequest, GetRoamingTaskResponse

from commons.OccupancyMap import OccupancyMap, Cell
from commons_msgs.msg import Goal


class RoamingTask():

    def __init__(self):

        if rospy.get_param('/use_cum_map', True) and rospy.get_param('/global_roam', True):
            self.threshold = 1
            kernel_size = 3
            self.kernel = np.ones([kernel_size, kernel_size])
            self.window_size = 8
            self.discovered_pub = rospy.Publisher('/discovered', Goal, queue_size=10)
            rospy.init_node('roaming_task')

            msg: OccupancyGrid = rospy.wait_for_message('/custom_layers/combined', OccupancyGrid)
            self.costmap_grid = numpy.asarray(msg.data).reshape([msg.info.width, msg.info.height])
            self.costmap = OccupancyMap.from_message(msg, with_data=False)

            msg = rospy.wait_for_message('/robot_0/move_base/global_costmap/costmap', OccupancyGrid)
            self.static_costmap_grid = numpy.asarray(msg.data).reshape([msg.info.width, msg.info.height])
            self.static_costmap = OccupancyMap.from_message(msg, with_data=False)

            rospy.Subscriber('/custom_layers/combined', OccupancyGrid, self.update_costmap)
            rospy.Timer(rospy.Duration(rospy.get_param('observed_cells_update_freq', 80)), self.publish_roaming_goals)
            rospy.Service('get_roaming_task', GetRoamingTask, self.get_roaming_goal)

            rospy.spin()

    def publish_roaming_goals(self, _=None):
        roaming_points = self.determine_critical_points(self.costmap_grid, self.window_size)
        for x, y in roaming_points:
            print(x, y)
            world_p = self.costmap.costmap2world(Cell(x, y))
            print(world_p)
            self.discovered_pub.publish(Goal(x=world_p.x, y=world_p.y, is_virtual=True,
                                             header=Header(stamp=rospy.get_rostime(), frame_id="map")))

    def get_roaming_goal(self, req: GetRoamingTaskRequest):
        world_p = Point()
        robot_position = Point(req.task.x, req.task.y, 0)
        roaming_points = self.determine_critical_points(self.costmap_grid, self.window_size)
        min_p = None
        min_dist = inf
        for x, y in roaming_points:
            world_p = self.costmap.costmap2world(Cell(x, y))
            d = math.sqrt((world_p.x - robot_position.x) ** 2 + (world_p.y - robot_position.y) ** 2)
            if d < min_dist:
                min_dist = d
                min_p = world_p
        if min_p is not None:
            return GetRoamingTaskResponse(task=Goal(None, world_p.x, world_p.y, True), success=True)
        else:
            return GetRoamingTaskResponse(task=Goal(None, 0, 0, True), success=False)

    def convolve(self, face, kernel) -> numpy.array:
        convolved = signal.fftconvolve(face, kernel, mode='same')
        convolved[convolved < self.threshold] = 0
        return convolved

    def determine_critical_points(self, costmap, radius: int) -> List[Tuple[int, int]]:

        convolved = self.convolve(costmap.T, self.kernel)

        i = 0
        while i <= 50:
            convolved[i][0] = 1000
            convolved[i][1] = 1000
            convolved[i][50] = 1000
            convolved[i][49] = 1000
            i += 1

        j = 0
        while j <= 50:
            convolved[0][j] = 1000
            convolved[50][j] = 1000
            convolved[1][j] = 1000
            convolved[49][j] = 1000
            j += 1
        numpy.set_printoptions(threshold=np.inf)
        print(convolved)
        result_points: List[Tuple[int, int]] = list()
        x_result = y_result = None
        result = np.where(convolved == 0)
        # print('num results: ', len(result[0]), " results: ", str(result))

        while self.all_points_iterated(convolved):
            result = np.where(convolved == 0)
            # print("iterate!!!!!!"+str(len(result[0])))
            indices = list(range(result[0].shape[0]))

            shuffle(indices)
            for i in indices:
                x = result[0][i]
                y = result[1][i]
                world_point = self.costmap.costmap2world(Cell(x, y))
                static_costmap_point = self.static_costmap.world2costmap(world_point)
                if self.static_costmap_grid.T[static_costmap_point.x, static_costmap_point.y] <= 0:
                    print(
                        f'{world_point} static map at {static_costmap_point} :'
                        f'{self.static_costmap_grid[static_costmap_point.x, static_costmap_point.y]}')
                    x_result = x
                    y_result = y
                    # print("@@@!!new point")
                    break
                else:
                    convolved[x][y] = 1000

            if x_result is not None:
                result_points.append((x_result, y_result))
                # print("result array index selected " + str(i))
                # print('Tuple of arrays returned x:', x_result, ' y:', y_result)
                convolved = self.slide(convolved, x_result, y_result, radius)
                # print('num results: ', len(result[0]), " results: ", str(result))

            else:
                break
        return result_points

    @staticmethod
    def slide(convolved, x: int, y: int, radius: int):
        i = -radius
        while i <= radius:
            j = -radius
            while j <= radius:

                if 0 < x + i <= 50 and 0 < y + j <= 50:
                    # print("x: "+str(x + i)+" y: "+str(y + j )+" "+str(convolved[x + i][y + j]))
                    convolved[x + i][y + j] = 1000
                elif x == 0 and y + j <= 50:
                    convolved[0][y + j] = 1000

                elif y == 0 and x + i <= 50:
                    convolved[0][x + i] = 1000

                elif y == 50 and x + i <= 50:
                    convolved[x + i][50] = 1000

                elif x == 50 and y + j <= 50:
                    convolved[y + j][50] = 1000

                j += 1
            i += 1
        return convolved

    @staticmethod
    def all_points_iterated(array):
        result = np.where(array == 0)
        # print('num results: ', len(result[0]), " results: ", str(result))

        return len(result[0])

    def update_costmap(self, msg: OccupancyGrid):
        self.costmap_grid = numpy.asarray(msg.data).reshape([msg.info.width, msg.info.height])
