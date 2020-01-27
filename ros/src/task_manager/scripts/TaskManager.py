#! /usr/bin/env python3

from functools import partial
from typing import List, Dict, Tuple

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Header

from task_manager_msgs.msg import ScheduleItem, ScheduleUpdated
from task_manager_msgs.srv import GetSchedule, GetScheduleResponse
from commons_msgs.msg import Goal
from task_allocator_msgs.msg import Confirmation

# Node name
NODE_NAME = 'task_manager'

# Topics and services
TASK_COMPLETED_TOPIC = 'task_completed'
SCHEDULE_UPDATED_TOPIC = 'schedule_updated'
CONFIRMATION_TOPIC = 'confirmation'
GET_SCHEDULE_SVC = 'get_schedule'
GOAL_ATTAINED_TOPIC = '/goal_attained'


class TaskManager:
    """
    Node responsible for managing the tasks to be executed by a robot
    """

    def __init__(self, robot_id: str, initial_x: float, initial_y: float):
        self.robot_id = robot_id
        self.location: Tuple[float, float] = (initial_x, initial_y)
        self.schedule_order: List[str] = []
        self.schedule: Dict[str, ScheduleItem] = dict()
        self.cost = 0
        self.total_seen_tasks = 0
        self.virtual_task_timeout = 60

        self.confirmation_sub = None
        self.robot_location_sub = None
        self.completed_sub = None
        self.agenda_srv = None
        self.completed_pub = None
        self.schedule_updated_pub = None

        self.__init_subscriptions()
        self.__init_publishers()

        rospy.loginfo(f"[{robot_id}][{NODE_NAME}] node is ready - "
                      f"\n\tlistening for new tasks on '{self.confirmation_sub.resolved_name}'"
                      f"\n\tlistening task completion on '{self.completed_sub.resolved_name}'"
                      f"\n\tpublishing schedule updates on '{self.schedule_updated_pub.resolved_name}'"
                      f"\n\tserving schedule requests at {self.agenda_srv.resolved_name}")
        rospy.spin()

    def __init_subscriptions(self):
        self.confirmation_sub = rospy.Subscriber(CONFIRMATION_TOPIC, Confirmation, self.add_to_schedule_cb)
        self.robot_location_sub = rospy.Subscriber('/' + self.robot_id + '/amcl_pose', PoseWithCovarianceStamped,
                                                   self.update_location_cb)
        self.completed_sub = rospy.Subscriber(TASK_COMPLETED_TOPIC, String, self.delete_task_cb)

    def __init_publishers(self):
        self.goal_attained_pub = rospy.Publisher(GOAL_ATTAINED_TOPIC, Goal, queue_size=1)
        self.schedule_updated_pub = rospy.Publisher(SCHEDULE_UPDATED_TOPIC, ScheduleUpdated, queue_size=1)

        self.agenda_srv = rospy.Service(GET_SCHEDULE_SVC, GetSchedule, self.get_schedule_cb)

    def __new_id(self) -> str:
        self.total_seen_tasks += 1
        return f'{self.robot_id}_task_{self.total_seen_tasks}'

    def update_location_cb(self, robot_location: PoseWithCovarianceStamped):
        self.location = (robot_location.pose.pose.position.x, robot_location.pose.pose.position.y)

    def delete_task_cb(self, task_id: String) -> None:
        rospy.logdebug(f"[{self.robot_id}][{NODE_NAME}] Removing task with id {task_id} from schedule")

        try:
            self.schedule_order.remove(task_id.data)
        except ValueError:
            pass
        try:
            self.cost -= self.schedule[task_id.data].edge_cost_in

            # Notify the Goal Manager about goal succeeded
            self.goal_attained_pub.publish(self.schedule[task_id.data].task)

            del self.schedule[task_id.data]
        except KeyError:
            pass

    def clear_virtual_task(self, task_id: str, _=None):
        self.delete_task_cb(String(task_id))
        self.schedule_updated_pub.publish(schedule=[self.schedule[item_id] for item_id in self.schedule_order])

    def add_to_schedule_cb(self, confirmation: Confirmation) -> None:
        rospy.logdebug(
            f"[{self.robot_id}][{NODE_NAME}]  Received confirmation of task at [{confirmation.Bid.task.x},"
            f"{confirmation.Bid.task.y}] addressed to {confirmation.robot_id}: {confirmation.Bid}")

        if self.robot_id != confirmation.robot_id:
            rospy.logdebug(f"[{self.robot_id}][{NODE_NAME}] Not the intended recipient.")
            return

        bid = confirmation.Bid
        schedule_item = ScheduleItem(task=bid.task, edge_cost_in=bid.edge_cost_in, id=self.__new_id())
        self.schedule[schedule_item.id] = schedule_item

        if bid.after or len(self.schedule_order) == 0:
            # flagged as "insert at the end" or agenda empty --> just append, no edge was removed
            # don't handle the case where the agenda has changed since the bid was made
            self.schedule_order.append(schedule_item.id)
            self.cost += schedule_item.edge_cost_in
            rospy.loginfo(f"[{self.robot_id}][{NODE_NAME}] task insertion at the end - new cost: {self.cost}")

        else:
            # insert into agenda at the appropriate point
            try:
                insert_at_index = self.schedule_order.index(bid.insert_at_id)
            except ValueError:
                # assume insertion point has already been processed --> insert at the front
                rospy.logwarn(
                    f"[{self.robot_id}][{NODE_NAME}] insertion point not found - assume already completed and "
                    f"inserting at front!")
                insert_at_index = 0
                bid.insert_at_id = self.schedule_order[0]
            insert_at_item = self.schedule[bid.insert_at_id]

            # update cost: remove old incoming edge, add cost for new edges and assign insertion point new edge cost
            self.cost = self.cost - insert_at_item.edge_cost_in + bid.edge_cost_out + schedule_item.edge_cost_in
            insert_at_item.edge_cost_in = bid.edge_cost_out

            self.schedule_order.insert(insert_at_index, schedule_item.id)
            rospy.loginfo(
                f"[{self.robot_id}][{NODE_NAME}] insert task task at {insert_at_index} - new cost: {self.cost},"
                f" new length: {len(self.schedule_order)}")

        self.schedule_updated_pub.publish(schedule=[self.schedule[item_id] for item_id in self.schedule_order])
        if confirmation.Bid.task.is_virtual:
            rospy.Timer(rospy.Duration(self.virtual_task_timeout), partial(self.clear_virtual_task, schedule_item.id))

    def get_schedule_cb(self, _) -> GetScheduleResponse:
        rospy.logdebug(f"[{self.robot_id}][{NODE_NAME}] Returning agenda of length {len(self.schedule_order)}")

        robot_location = ScheduleItem(edge_cost_in=0, id='robot_location',
                                      task=Goal(
                                          header=Header(frame_id='map', stamp=rospy.get_rostime()),
                                          x=self.location[0], y=self.location[1], is_virtual=False))
        return GetScheduleResponse(
            schedule=[robot_location] + [self.schedule[item_id] for item_id in self.schedule_order],
            cost=self.cost)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    robot_id = rospy.get_param('~robot_id', 'robot_0')
    initial_x = rospy.get_param('~initial_x', 'robot_0')
    initial_y = rospy.get_param('~initial_y', 'robot_0')

    TaskManager(robot_id, initial_x, initial_y)