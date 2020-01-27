#! /usr/bin/env python3

from typing import List

import rospy
from actionlib import ActionClient, Header, ClientGoalHandle, CommState
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

from task_manager_msgs.msg import ScheduleItem, ScheduleUpdated
from commons_msgs.msg import Goal

from roaming_task.srv import GetRoamingTaskResponse, GetRoamingTask

# Node name
NODE_NAME = 'task_executor'

# Topics and services
TASK_COMPLETED_TOPIC = "task_completed"
SCHEDULE_UPDATED_TOPIC = "schedule_updated"

ROAMING_TASK_ID = "ROAM"

# This allow up to 5 robots to roam and discover new tasks without any of the
# costmaps etc. being up and running.
# Choose roaming_mode FIXED_POINTS to use
# WARNING: these points are chosen based on domain knowledge of the map
# that shouldn't be available to the robots
roaming_goals = {
    'robot_1': [Point(4, 2.5, 0), Point(-4, -2.5, 0)],
    'robot_0': [Point(-4, 4.2, 0), Point(0, -4.5, 0)],
    'robot_2': [Point(-1.5, 4, 0), Point(3, -3, 0)],
    'robot_3': [Point(1.5, 4, 0), Point(2, 0, 0)],
    'robot_4': [Point(1.5, 3, 0), Point(-2.2, -1, 0)]
}
r_counter = 0


class TaskExecutor:
    """
    Class responsible for completing all tasks in the robot's agenda.
    Agenda is provided by an AgendaManager.
    Communication with move_base is done through its actionlib interface.

    Parameters:
        - roaming_mode: one of the following (default: SMART)
            FIXED_POINTS: use predefined goals for up to 4 robots (see above)
            SMART: use externally provided roaming goals
                    (eg. based on the costmaps to determine regions with high task probability)
                    requires service to get roam goal at /get_roaming_task
            NO_ROAM: robots don't move at all if they don't have a task
    Topics:
        - subscribed:
            agendaUpdated: AgendaUpdated - Receive full agenda updates
        - publishes:
            taskCompleted: String - Notify upon task completion by agenda_item id
    """

    action_namespace = rospy.get_namespace() + "move_base"

    def __init__(self):
        self.schedule: List[ScheduleItem] = list()
        self.action_client = ActionClient(self.action_namespace, MoveBaseAction)
        self.current_goal: ClientGoalHandle = None

        self.roam_mode = rospy.get_param(f'{rospy.get_name()}/roaming_mode', 'FIXED_POSITIONS')

        self.robot_prefix = rospy.get_namespace().replace('/', '')
        self.current_task_id = 'NONE'

        self.agenda_updated_sub = None
        self.roaming_point_srv = None
        self.task_completed_pub = None

        self.__init_subscriptions()
        self.__init_publishers()

        if not self.action_client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr(f"Could not connect to action server at {self.action_namespace}")
            raise rospy.ROSException(f"[{self.robot_prefix}][{NODE_NAME}] Could not connect to action server")

        rospy.loginfo(
            f"[{self.robot_prefix}][{NODE_NAME}] node is ready - "
            f"publishing task completion on '{self.task_completed_pub.resolved_name}', "
            f"listening for schedule updates on '{self.schedule_updated_sub.resolved_name}', "
            f"connected to move_base at {self.action_client.ns}")

        if self.roam_mode not in ["FIXED_POSITIONS", "NO_ROAM"]:
            rospy.wait_for_service('/get_roaming_task')

        self.current_task_id = "NONE"
        self.send_next()

        rospy.spin()

    def __init_subscriptions(self):
        self.schedule_updated_sub = rospy.Subscriber(SCHEDULE_UPDATED_TOPIC, ScheduleUpdated, self.schedule_updated_cb)

        self.roaming_point_srv = rospy.ServiceProxy('/get_roaming_task', GetRoamingTask)

    def __init_publishers(self):
        self.task_completed_pub = rospy.Publisher(TASK_COMPLETED_TOPIC, String, queue_size=10)

    def mark_current_task_completed(self) -> None:
        """
        Notify subscribers that the current task is completed.
        Task is identified by its schedule_item id (as set by Task Manager)
        """

        item_id = self.schedule.pop(0).id
        rospy.loginfo(f"[{self.robot_prefix}][{NODE_NAME}] confirm completion of task {item_id}")
        self.task_completed_pub.publish(item_id)

    def schedule_updated_cb(self, schedule_updated_message: ScheduleUpdated) -> None:
        """
        Callback for the UPDATED_TOPIC subscription.
        If the new agenda has a different first task than the current agenda,
        the current task - if any - is preempted and the new first task is sent as the new goal
        else just the list is updated.

        :param schedule_updated_message: Full update of agenda, sent by AgendaManager
        """

        rospy.logdebug(f"[{self.robot_prefix}][{NODE_NAME}]"
                       f" got agenda of length {len(schedule_updated_message.schedule)}")
        self.schedule = schedule_updated_message.schedule

        if len(self.schedule) == 0 or self.current_task_id != self.schedule[0].id:
            rospy.logdebug(f"[{self.robot_prefix}][{NODE_NAME}] New first node - updating goal")
            if self.current_goal is not None:
                self.current_goal.cancel()
            self.send_next()

    def action_client_completed_callback(self, goal_handle: ClientGoalHandle) -> None:
        """
        Callback to be called for all goal state tranisitions.
        All transitions except to the DONE state are ignored.

        If the task was not aborted or preempted it is marked as completed and the robot moves on to the next task.
        If the task was completed with a non-succeeding state, it will still be marked as completed and a warning is
        printed.

        See also: http://docs.ros.org/api/actionlib/html/classactionlib_1_1action__client_1_1CommStateMachine.html

        :param goal_handle: Actionlib GoalHandle for the task
        """
        rospy.logdebug(f"[{self.robot_prefix}][{NODE_NAME}] task {self.current_task_id}: stat change of task to "
                       f"{goal_handle.get_goal_status()} - {goal_handle.get_goal_status_text()}")
        if goal_handle.get_comm_state() == CommState.DONE:
            if goal_handle.get_goal_status() != GoalStatus.PREEMPTED:
                if goal_handle.get_goal_status() != GoalStatus.SUCCEEDED:
                    rospy.logwarn(f"[{self.robot_prefix}][{NODE_NAME}]"
                                  f"Task completed with non-succeeding state: {goal_handle.get_goal_status_text()}")
                if self.current_task_id != ROAMING_TASK_ID and self.current_task_id != "NONE":
                    self.mark_current_task_completed()
                self.send_next()
            else:
                rospy.logwarn(f"[{self.robot_prefix}][{NODE_NAME}]"
                              f"Task preempted")

    def send_next(self, timer_context=None) -> None:
        """
        Send the next task or a roaming target - if any - as a goal to the move_base
        """

        # if called from timer only execute if robot is still waiting for a goal
        if timer_context is None or (len(self.schedule) == 0 and self.current_goal.get_goal_status() == 3):
            if len(self.schedule):
                goal_point = Point(self.schedule[0].task.x, self.schedule[0].task.y, 0)
            else:
                goal_point = self.get_roaming_point()

            if goal_point is not None:
                goal = MoveBaseGoal(PoseStamped(Header(frame_id='map'), Pose(goal_point, Quaternion(0, 0, 0, 1))))
                self.current_task_id = self.schedule[0].id if len(self.schedule) > 0 else ROAMING_TASK_ID

                rospy.loginfo(
                    f"[{self.robot_prefix}][{NODE_NAME}] task {self.current_task_id}: Sending goal to move_base")
                rospy.logdebug(f"[{self.robot_prefix}][{NODE_NAME}] Goal was: {goal}")

                self.current_goal = self.action_client.send_goal(goal, self.action_client_completed_callback)
            else:
                # if no task or roaming goal is available try again in 5 seconds.
                self.current_task_id = "NONE"
                rospy.Timer(rospy.Duration(5), self.send_next)

    def get_roaming_point(self):
        if self.roam_mode == "FIXED_POSITIONS":
            global r_counter
            r_counter += 1
            return roaming_goals[self.robot_prefix][r_counter % 2]
        elif self.roam_mode == "NO_ROAM":
            return None
        else:
            start: PoseWithCovarianceStamped = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
            rospy.logwarn(f'[{self.robot_prefix}]get roam point!')
            resp: GetRoamingTaskResponse = self.roaming_point_srv(
                Goal(None, start.pose.pose.position.x, start.pose.pose.position.y, True))
            if resp.success:
                roam_task = resp.task
                return Point(roam_task.x, roam_task.y, 0)
            else:
                return None


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    TaskExecutor()
