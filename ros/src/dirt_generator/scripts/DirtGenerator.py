#! /usr/bin/env python3

import random
import numpy
import threading
import math
from typing import List

import rospy
import rospkg
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from std_msgs.msg import Header

from goal_manager_msgs.msg import GoalObject, GoalObjectList, DirtModel
from commons.OccupancyMap import OccupancyMap, Cell

# Node name
NODE_NAME = 'dirt_generator'

# Topics and services
# BASE_MAP_TOPIC = '/robot_0/map'
BASE_MAP_TOPIC = 'modified_occupancy_grid'
GOALS_TOPIC = 'dirt_and_goals'
NEW_DIRT_TOPIC = 'new_dirt'

# Gazebo model params
DIRT_MODEL_NAME = "dirt_object_undetected.sdf"
DIRT_MODEL_PACKAGE = "dirt_generator"

# Sedd to be used during simulation
SEED = 120

# Limits on the world map, that is, where the robot cannot move
X_MIN_IN = -5.0
X_MAX_IN = 5.0
Y_MIN_IN = -5.0
Y_MAX_IN = 5.0

# Min and max values for random timers
TIME_MIN = 10
TIME_MAX = 10

# Min and max trust level, that is, amount of confidence about the dirt observation
TRUST_MIN = 100
TRUST_MAX = 100

# Debug control constant
DEBUG_ON = True


class DirtGenerator:
    def __init__(self, seed, spawn_interval):
        self.position_map: List[Point] = []
        self.spawn_number: int = 1
        self.robot_size: float = 0.105 * 2
        self.dirt_pos_tolerance: float = 0.25
        self.occupancy_map: OccupancyGrid
        self.active_dirt_list: List[GoalObject] = list()

        self.model_pub = None
        self.scan_sub = None
        self.goals_sub = None
        self.seed = seed
        self.time_interval_min = spawn_interval
        self.time_interval_max = spawn_interval

        self.occupancy_map = None

        self.__init_publishers()
        self.__init_subscribers()

        rospy.loginfo(f"[{NODE_NAME}] node is ready - "
                      f"\n\tlistening for active goals on '{self.goals_sub.resolved_name}"
                      f"\n\tpublishing new random dirt to '{self.model_pub.resolved_name}'")

    def __init_subscribers(self):
        self.goals_sub = rospy.Subscriber(GOALS_TOPIC,
                                          GoalObjectList, self.__dirt_and_goals_list_cb)

    def __init_publishers(self):
        self.model_pub = rospy.Publisher(NEW_DIRT_TOPIC, DirtModel, queue_size=100)

    def __dirt_and_goals_list_cb(self, combined_list):
        # Save the received list with all currently active dirt and goals (from topic all_current_dirt_and_goals)
        active_detected_dirt_list = list(combined_list.goal_list)
        self.active_dirt_list = list(active_detected_dirt_list)

    def __comparing_points(self, point1, point2) -> bool:
        """
        Compares two Points and returns true if they are identical (same position with some tolerance)
        """
        return (abs(point1.x - point2.x) <= self.dirt_pos_tolerance and abs(
            point1.y - point2.y) <= self.dirt_pos_tolerance)

    def __check_for_duplicates(self, point) -> bool:
        """
        Goes through the list with all currently active dirt positions and compare their positions with the given
        position. Returns true if there is a duplicate, false otherwise
        """

        # Check all already published (active) dirt objects (stored and received from the goal_list)
        for dirt in list(self.active_dirt_list):
            if self.__comparing_points(point, dirt.pose.position):
                return True

        return False

    def __get_cell_index(self, x, y) -> int:
        cell_x = int((x - self.occupancy_map.origin.x) / self.occupancy_map.resolution)
        cell_y = int((y - self.occupancy_map.origin.y) / self.occupancy_map.resolution)

        index = cell_x + cell_y * self.occupancy_map.width

        return index

    def __is_occupied(self, x, y) -> bool:
        """
        Check if the cell at position (x, y) is occupied or not (with a static obstacle like a wall)
        """

        cell = self.occupancy_map.world2costmap(self.occupancy_map.costmap2world(Cell(x, y)))
        index = self.occupancy_map.to_costmap_index(cell)

        return self.occupancy_map.grid[index] != 0

    def __is_occupied_(self, x, y) -> bool:

        index = self.__get_cell_index(x, y)

        return self.occupancy_map.grid[index] != 0

    def __has_occupied_neighbors(self, x, y) -> bool:
        """
        Checks if the neighbor cells (mask: radius of robot size) of the cell at the given position (x,y) are occupied
        Mask is probably larger than needed, but it is okay (safer)
        """

        # Robot radius in cells (according to OccupancyGrid)
        robot_radius = int(math.ceil((self.robot_size / 2) / self.occupancy_map.resolution))

        center_index = self.__get_cell_index(x, y)

        # Only check the cells in the square around the center with edge length of twice the robot_radius (the center
        # cell can be ignored, that is why robot_radius-1)
        for r in range(-(robot_radius - 1), (robot_radius - 1)):

            # Is actually not needed because we assume that the robot is symmetrical (square circle)
            for c in range(-(robot_radius - 1), (robot_radius - 1)):

                # Now refine the initial square with transforming the mask to a circle (nearly)
                if math.floor(math.sqrt(r ** 2 + c ** 2)) <= robot_radius:
                    current_index = center_index + c + r * self.occupancy_map.width

                    # if in this circle one of the cells is occupied (!=0), then the given cell is not possible (
                    # return true)
                    if self.occupancy_map.grid[current_index] != 0:
                        return True

        # Only if all cells in the circle mask are free, then the given cell is possible as dirt center
        return False

    def __get_dirt_candidate_cells(self):
        while True:
            if self.occupancy_map:
                # As soon as map and metadata is received (!=0.0), create a static list with all possible positions

                x_min = self.occupancy_map.origin.x
                y_min = self.occupancy_map.origin.y
                x_max = x_min + self.occupancy_map.height * self.occupancy_map.resolution
                y_max = y_min + self.occupancy_map.width * self.occupancy_map.resolution
                x_step = y_step = self.occupancy_map.resolution

                # Take always the center position of the grid cells
                for x in numpy.arange(x_min + x_step/2, x_max - x_step/2, x_step):

                    # Take always the center position of the grid cells
                    for y in numpy.arange(y_min + y_step/2, y_max - y_step/2, y_step):

                        # Check if it is inside the movement area of the robots
                        if (X_MIN_IN <= x <= X_MAX_IN) and (Y_MIN_IN <= y <= Y_MAX_IN):

                            if not self.__is_occupied_(x, y):
                                self.position_map.append(Point(x=x, y=y, z=0.0))

                break

            # Sleep one second until self.position_map can be created (occupancy grid, etc. was received)
            #rospy.sleep(1)

    def __generate_point_based_on_prob(self) -> Point:
        """
        Generates a random point, based on probabilities (map/distribution)
        Returns random point of type Point(x, y, z)
        """

        # Use a standard distribution (provided by libs/extern formulas) In this case: Beta distribution
        # with alpha=2 and beta=2 (near to normal/Gaussian): ATTENTION: the distribution is applied to a position
        # list, which goes row by row through the map. That means the hot spot (in case of Gaussian) is not a perfect
        # circle in the middle of the map, but the complete rows in the middle of the map (also their boundary cells)
        # We tested it and the distribution is good for our purpose, but keep in mind: it is not a circle in the
        # center!
        index = 0
        possible = False
        while not possible:
            index = int(random.betavariate(2, 2) * len(self.position_map))
            x = self.position_map[index].x
            y = self.position_map[index].y

            # Check for occupied neighbors and only add the position if also the neighboring cells are free (the
            # robot needs some space the reach it), otherwise generate a new one and test it
            if not self.__has_occupied_neighbors(x, y):
                # If actual spawning of dirt is enabled, then it should also be checked while generating objects
                # if another dirt object is already at this position (if so, the generation has to be repeated!)
                if not self.__check_for_duplicates(Point(x, y, 0.0)):
                    possible = True
                else:
                    rospy.loginfo(
                        rospy.get_caller_id() + "\n\n\tGenerated dirt at (%f | %f) was refused due to already "
                                                "active dirt at this position (duplicate). Generating next "
                                                "one...\n" % (
                            x, y))
            else:
                rospy.loginfo(
                    rospy.get_caller_id() + "\n\n\tGenerated dirt at (%f | %f) was refused due to occupied neighbor "
                                            "cells. Generating next one...\n" % (
                        x, y))

        return self.position_map[index]

    def __spawn_dirt(self, dirt: GoalObject):
        """
        Spawns dirt object in the map at the position which was generated for the dirt delivered via the input
        parameter of this function
        """

        # Init service
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        name = "dirt_" + str(self.spawn_number)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(DIRT_MODEL_PACKAGE)
        path = pkg_path + "/" + DIRT_MODEL_NAME
        with open(path, "r") as data:
            model = data.read()
        robot = ""  # can be left empty
        pose = dirt.pose
        frame = ""  # empty or "world" or "map"

        # save name of model combined with position for future deletion (transmit it to goal_list where this is
        # executed)
        new_dirt_model = DirtModel(header=Header(stamp=rospy.get_rostime(), frame_id="map"), name=name, pose=pose)
        self.model_pub.publish(new_dirt_model)

        # Spawn it
        spawn_model(name, model, robot, pose, frame)
        rospy.loginfo(rospy.get_caller_id() + "\n\n\tNew dirt was spawned\n")
        self.spawn_number += 1

    def generation_process(self):
        """
        Creates a random dirt until the node is shut down
        """

        # Sleep at the beginning because not everything is set up (nodes, topics)
        rospy.sleep(1)

        index = 0
        while not rospy.is_shutdown():
            # Create an (increasing) index, a random trust value and a random position for the new dirt
            index += 1
            trust_value = random.randint(TRUST_MIN, TRUST_MAX)
            pose = Pose(position=self.__generate_point_based_on_prob(),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            # Position/Point should already be in an empty (not occupied) cell, because it is only randomly searched
            # on already free positions

            # Combine everything in the new object
            goal = GoalObject(index, pose, trust_value, 0)
            rospy.loginfo("\n\n\t(%d)Dirt/Goal generated: [ID: %d, (%f,%f),trust: %d]\n" % (self.seed,
                goal.id, goal.pose.position.x, goal.pose.position.y, goal.trust_value))

            # Spawn the dirt (if it is enabled)
            self.__spawn_dirt(goal)

            # Sleep rest of the (random defined) time
            # sleep_time = random.randint(TIME_MIN, TIME_MAX)
            sleep_time = random.randint(self.time_interval_min, self.time_interval_max)
            rospy.loginfo("\n\n\tDirt generation will sleep now for %d seconds.\n" % sleep_time)
            rospy.sleep(sleep_time)

    def dirt_generator(self):
        r = rospy.Rate(1)
        # while not rospy.is_shutdown():
        rospy.loginfo(f"[{NODE_NAME}] \n\tWaiting for occupancy map.")
        self.occupancy_map = OccupancyMap.from_message(rospy.wait_for_message(BASE_MAP_TOPIC, OccupancyGrid))
        # self.dirt_pos_tolerance = self.occupancy_map.resolution

        self.__get_dirt_candidate_cells()

        # Seed can be set with a parameter
        # random.seed(SEED)
        random.seed(self.seed)

        # Start generating+spawning dirt
        thread1 = threading.Thread(target=self.generation_process)
        thread1.start()

        rospy.spin()


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    seed_ = rospy.get_param(f'~seed', 100)
    spawn_interval_ = rospy.get_param(f'~spawn_interval', 10)
    dg = DirtGenerator(seed_, spawn_interval_)

    dg.dirt_generator()
