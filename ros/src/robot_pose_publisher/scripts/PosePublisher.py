#! /usr/bin/env python3

from typing import List, Dict
from threading import Lock
import re

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point

from robot_pose_publisher.msg import RobotLocation, RobotLocations

# from robot_pose_publisher.srv import GetLocations, GetLocationsResponse

# Node name
NODE_NAME = 'pose_publisher'

# Topics and services
LOCATIONS_PUB = 'robots_locations'

# GET_LOCATIONS_SVC = 'robots_locations'


class PosePublisher:
    def __init__(self):
        self.subscription_lock = Lock()
        self.robots_locations_lock = Lock()
        self.robots_locations: Dict[str, Pose] = dict()

        self.location_subscriptions: Dict[str, rospy.Subscriber] = dict()

        self.locations_pub = rospy.Publisher(LOCATIONS_PUB, RobotLocations, queue_size=100)
        # self.agenda_srv = rospy.Service(GET_LOCATIONS_SVC, GetLocations, self.get_locations_cb)

        rospy.Timer(rospy.Duration(30), self.update_pose_subscriptions)
        self.update_pose_subscriptions()

        rospy.spin()

    def update_location(self, robot_location: PoseWithCovarianceStamped, source_topic: str) -> None:
        self.robots_locations[source_topic] = robot_location.pose.pose
        self.__publish_locations()

    def update_pose_subscriptions(self, _=None) -> None:
        reg = re.compile('/[^/]+/amcl_pose')
        current_topics = {top[0] for top in rospy.client.get_published_topics() if
                          reg.match(top[0]) and top[1] == 'geometry_msgs/PoseWithCovarianceStamped'}
        # rospy.loginfo(f"[{NODE_NAME}] Now listening for robot poses on: {', '.join(current_topics)}")

        # locations and subscriptions for no longer active pose topics are deleted
        with self.subscription_lock:
            for topic in self.location_subscriptions.keys() - current_topics:
                self.location_subscriptions[topic].unregister()
                del self.location_subscriptions[topic]
                del self.robots_locations[topic]

            # new pose topic discovered -> subscribe to it
            for topic in current_topics - self.location_subscriptions.keys():
                self.location_subscriptions[topic] = rospy.Subscriber(topic, PoseWithCovarianceStamped,
                                                                      callback=self.update_location,
                                                                      callback_args=topic)
                self.robots_locations[topic] = None

    def __publish_locations(self):
        #rospy.loginfo(f"[{NODE_NAME}] Publishing robots locations")
        # for robot in self.robots_locations:
        #     rospy.loginfo(
        #         f"\n\tRobot {robot} at "
        #         f"{self.robots_locations[robot].position.x},{self.robots_locations[robot].position.y}")

        r_locations: List[RobotLocation] = list()
        for robot in self.robots_locations:
            if self.robots_locations[robot] is not None:
                r_locations.append(RobotLocation(robot_id=robot[robot.find('/')+1:robot.rfind('/')], pose=self.robots_locations[robot]))
        robot_locations = RobotLocations()
        robot_locations.locations = r_locations
        self.locations_pub.publish(robot_locations)

    # def get_locations_cb(self, dummy: bool): response: GetLocationsResponse = GetLocationsResponse(locations=[
    # self.robots_locations[robot] for robot in self.robots_locations]) return response


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    PosePublisher()
