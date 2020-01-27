#! /usr/bin/env python3

from typing import List, Union, NamedTuple
from math import sqrt, inf

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse

from task_manager_msgs.msg import ScheduleItem
from commons_msgs.msg import Goal
from bidder_msgs.msg import Bid
from task_manager_msgs.srv import GetSchedule, GetScheduleResponse, GetScheduleResponse

# Node name
NODE_NAME = 'bidder'

# Topics and services
BID_TOPIC = 'bid'
NEW_GOAL_TOPIC = 'new_goal'
COSTMAP_TOPIC = 'move_base/global_costmap/costmap'
SCHEDULE_SVC = 'get_schedule'
PLAN_SVC = 'move_base/GlobalPlanner/make_plan'


class CellCoordinate(NamedTuple):
    x: int
    y: int


class Bidder:
    """
    Node responsible for bidding the total cost of adding a discovered task according to the robot' schedule

    The insertion of the new task is considered before and after the point on the existing schedule with the lowest
    euclidean distance to the new task. Only the better bid out of these two will be published.

    The calculated costs are based on the costmaps of the planner and the distance. For each path between tasks the
    cost is:
        total cost of traversed cells + (number of traversed cells)^2
    """

    def __init__(self, robot_id: str, adaptive_scheduling: bool):
        self.robot_id: str = robot_id
        self.adaptive_scheduling = adaptive_scheduling
        self.costmap_width: int = 0
        self.costmap_height: int = 0
        self.costmap: List[int] = list()
        self.origin: Point = Point()
        self.resolution: float = 0
        self.bid_pub = None
        self.new_goal_sub = None
        self.costmap_sub = None
        self.get_plan = None
        self.get_schedule = None

        self.__init_publishers()
        self.__init_subscriptions()

        self.get_schedule = rospy.ServiceProxy(SCHEDULE_SVC, GetSchedule)
        self.get_plan = rospy.ServiceProxy(PLAN_SVC, GetPlan)

        rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] node is ready - "
                      f"\n\tlistening for new goals on '{self.new_goal_sub.resolved_name}"
                      f"\n\tpublishing bids on '{self.bid_pub.resolved_name}'")

        rospy.wait_for_service(SCHEDULE_SVC)

        rospy.spin()

    def __init_subscriptions(self):
        self.costmap_sub = rospy.Subscriber(COSTMAP_TOPIC, OccupancyGrid, self.update_costmap_cb)
        self.new_goal_sub = rospy.Subscriber(NEW_GOAL_TOPIC, Goal, self.new_task_cb)

    def __init_publishers(self):
        self.bid_pub = rospy.Publisher(BID_TOPIC, Bid, queue_size=10)

    def update_costmap_cb(self, msg: OccupancyGrid):
        self.origin = msg.info.origin.position
        self.resolution = msg.info.resolution
        self.costmap_width = msg.info.width
        self.costmap_height = msg.info.height
        self.costmap = msg.data

    @staticmethod
    def __get_closest_schedule_item(schedule: List[ScheduleItem], task: Goal) -> int:
        """
        Returns the index of the closest item in the schedule plan to the given task

        :param schedule: the list of items in the schedule
        :param task: the reference task
        :return: index of the closest item in the schedule
        """
        current_min = inf
        current_idx = 0

        for i, item in enumerate(schedule):
            distance = sqrt((task.x - item.task.x) ** 2 + (task.y - item.task.y) ** 2)
            if distance <= current_min:
                current_min = distance
                current_idx = i

        return current_idx

    def __to_costmap_index(self, point: CellCoordinate) -> int:
        """
        Calculates and returns the index to the given coordinate within the costmap

        :param point: the coordinate
        :return: the index
        """
        return point.y * self.costmap_width + point.x

    def __get_cost(self, from_task: Goal, to_task: Goal) -> Union[int, float]:
        """
        Calculates the cost for the robot to go from task from_task to to_task.
        Uses the GlobalPlanner to get an accurate path based on the current costmap. The total is:
            cost of path + number of traversed_cells squared

        :param from_task: starting point of the cost request
        :param to_task:  ending point of the cot request
        :return: cost of path or inf if no path was not found
        """

        req = GetPlanRequest()
        req.start = PoseStamped()

        req.start.header.frame_id = 'map'
        req.goal.header.frame_id = 'map'

        req.start.pose = Pose(position=Point(x=from_task.x, y=from_task.y))
        req.goal.pose = Pose(position=Point(x=to_task.x, y=to_task.y))

        resp: GetPlanResponse = self.get_plan(req)
        if len(resp.plan.poses) == 0:
            # If no plan can be found, return max value of uint32
            return inf

        # Find unique set of traversed cells
        traversed_cells = {CellCoordinate(round((point.pose.position.x - self.origin.x) / self.resolution),
                                          round((point.pose.position.y - self.origin.y) / self.resolution)) for point in
                           resp.plan.poses}
        cost = sum(map(lambda cell: self.costmap[self.__to_costmap_index(cell)], traversed_cells)) + len(
            traversed_cells) ** 2

        return cost

    def new_task_cb(self, task: Goal) -> None:
        """
        Initiates the bidding process when a new task is received from the Goal Manager node.

        To bid a task cost, the following steps are followed:
            1. Find the appropriate insertion point (or end of the list if the schedule is clear)
            2. Calculate the cost before and after the insertion point
            3. Create a proposal for the lower calculated cost and send the bid to the Task Allocator

        :param task: the new task to handle
        :return: None
        """

        scheduleResp: GetScheduleResponse = self.get_schedule()
        schedule: List[ScheduleItem] = scheduleResp.schedule
        cost = scheduleResp.cost

        rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] generating proposal for inserting [{task.x},{task.y}] "
                      f"into schedule of length {len(schedule) - 1} with cost {cost}")

        # If adaptive scheduling is disabled, always insert the task at the end of the schedule
        # Recall that the schedule always has a virtual task to move around the map, therefore, the length of the
        # schedule is always at least 1

        # rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] adaptive scheduling? {self.adaptive_scheduling}")

        if not self.adaptive_scheduling:
            in_cost = len(schedule)
            # in_cost = self.__get_cost(schedule[len(schedule)-1].task, task)
            if in_cost < inf:
                rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] adding at the end of schedule schedule - "
                              f"proposal append with cost {in_cost}")
                self.bid_pub.publish(
                    Bid(robot_id=self.robot_id, task=task, cost=in_cost, edge_cost_in=in_cost, after=True))
            else:
                rospy.logwarn(f"[{self.robot_id}][{NODE_NAME}] could not find path to propose insertion")
            return None
        # Otherwise, use adaptive scheduling
        else:
            if len(schedule) == 1:
                # Contains only a task, then append; cost is equal to edge cost, as it is the only edge in the schedule
                in_cost = self.__get_cost(schedule[0].task, task)
                if in_cost < inf:
                    rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] empty schedule - proposal append with cost {in_cost}")
                    self.bid_pub.publish(
                        Bid(robot_id=self.robot_id, task=task, cost=in_cost, edge_cost_in=in_cost, after=True))
                else:
                    rospy.logwarn(f"[{self.robot_id}][{NODE_NAME}] could not find path to propose insertion")
                return None

            insert_at_idx = self.__get_closest_schedule_item(schedule, task)

            # Insertion is either before
            if insert_at_idx == 0:
                rospy.logdebug(
                    f"[{self.robot_id}][{NODE_NAME}] closest point was robot - not considering 'before' insertion")
                before = {"cost": inf}
            else:
                before = {"insert_at_id": schedule[insert_at_idx].id,
                          "edge_cost_in": self.__get_cost(schedule[insert_at_idx - 1].task, task),
                          "edge_cost_out": self.__get_cost(task, schedule[insert_at_idx].task),
                          }
                before["cost"] = cost + before["edge_cost_in"] + before["edge_cost_out"] - schedule[
                    insert_at_idx].edge_cost_in

            # or after - here, we have to differentiate between appending and insertion
            should_append = insert_at_idx == len(schedule) - 1
            after = {
                "after": should_append,
                "edge_cost_in": self.__get_cost(schedule[insert_at_idx].task, task),
                "edge_cost_out": 0 if should_append else self.__get_cost(task, schedule[insert_at_idx + 1].task)
            }
            after["cost"] = cost + after["edge_cost_in"] + after["edge_cost_out"] \
                            - (0 if should_append else schedule[insert_at_idx + 1].edge_cost_in)
            if not should_append:
                after["insert_at_id"] = schedule[insert_at_idx + 1].id

            if before['cost'] < inf or after['cost'] < inf:
                rospy.loginfo(
                    f"[{self.robot_id}][{NODE_NAME}] propose: insertion at '{schedule[insert_at_idx].id}'"
                    f" - costs:  after: {after['cost']} "
                    f"{' (append)' if should_append else ''},"
                    f" before: {before['cost']}")
                self.bid_pub.publish(
                    Bid(task=task, robot_id=self.robot_id, **(before if before["cost"] <= after["cost"] else after)))
            else:
                rospy.logwarn(f"[{self.robot_id}][{NODE_NAME}] could not find path to propose insertion")


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    robot_id = rospy.get_param('~robot_id', 'robot_0')
    adaptive_scheduling = rospy.get_param('~adaptive_scheduling', 'true')

    bidder = Bidder(robot_id, adaptive_scheduling)
