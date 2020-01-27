#! /usr/bin/env python3

from typing import Tuple, Dict
import re
from dataclasses import dataclass
from functools import partial

import rospy
import rosnode

from task_allocator_msgs.msg import Confirmation
from bidder_msgs.msg import Bid

# Node name
NODE_NAME = 'task_allocator'

# Topics and services
BID_TOPIC = 'bid'
CONFIRMATION_TOPIC = 'confirmation'


@dataclass
class StoredBid:
    bid: Bid
    num_bids: int
    active: bool = True


class TaskAllocator:
    """
    Class responsible for assigning tasks to the best bid (lowest cost).
    Tasks are identified by their coordinates

    Tasks are either confirmed once all bids have been heard or after a timeout runs out.
    """
    bid_store: Dict[Tuple[float, float, int], StoredBid] = dict()

    def __init__(self, no_of_robots=2, timeout_sec=3, refresh_robot_number_interval=60):
        self.num_robots = no_of_robots
        self.timeout = timeout_sec
        self.confirm_pub = None
        self.bid_sub = None

        self.__init_subscribers()
        self.__init_publishers()

        rospy.loginfo(f"[{NODE_NAME}] node is ready - "
                      f"\n\tlistening for new bids on '{self.bid_sub.resolved_name}"
                      f"\n\tpublishing task allocations to '{self.confirm_pub.resolved_name}'")

        rospy.Timer(rospy.Duration(refresh_robot_number_interval), self.update_no_robots)
        rospy.spin()

    def __init_publishers(self):
        self.confirm_pub = rospy.Publisher(CONFIRMATION_TOPIC, Confirmation, queue_size=10)

    def __init_subscribers(self):
        self.bid_sub = rospy.Subscriber(BID_TOPIC, Bid, self.bid_cb)

    def update_no_robots(self, _) -> None:
        reg = re.compile('/[^/]+/bidder')
        self.num_robots = len({name for name in rosnode.get_node_names() if reg.match(name)})
        rospy.loginfo(f"[{NODE_NAME}] updated number of robots to: {self.num_robots}")

    def __store_bid(self, coordinate: Tuple[float, float, int], bid: Bid) -> None:
        """
        Stores a bid: either create a new entry, add the bid if its cost is better or simply increment the
        seen counter.

        :param coordinate: identifying coordinate of bid
        :param bid: the bid
        """

        # new bid - put in store and start timer
        if coordinate not in self.bid_store:
            stored_bid = StoredBid(
                bid=bid,
                num_bids=1,
            )
            self.bid_store[coordinate] = stored_bid
            rospy.Timer(rospy.Duration(self.timeout), partial(self.confirm, coordinate), oneshot=True)
        # task is already timed out -> ignore bid
        elif not self.bid_store[coordinate].active:
            return
        # bid is better than previous bids
        elif self.bid_store[coordinate].bid.cost > bid.cost:
            self.bid_store[coordinate].bid = bid
            self.bid_store[coordinate].num_bids += 1
        # bid is worse than previous -> just increment counter
        else:
            self.bid_store[coordinate].num_bids += 1

    def bid_cb(self, bid: Bid) -> None:
        """
        Callback for ROS subscription. Store bid and send out confirmation if all robots proposed for that task
        :param bid: the bid
        """

        coordinates: Tuple[float, float, int] = (bid.task.x, bid.task.y, bid.task.header.stamp.secs)
        self.__store_bid(coordinates, bid)
        if self.bid_store[coordinates].num_bids >= self.num_robots:
            self.confirm(coordinates)

    def confirm(self, coordinates: Tuple[float, float, int], _=None) -> None:
        """
        Sends out confirmation with current best bid and closes the Task afterwards
        :param coordinates: identifier (coordinates) of the task that should be confirmed
        :param _: unused param that is only passed when this is called using rospy.Timer
        """

        if self.bid_store[coordinates].active:
            rospy.loginfo(
                f"[{NODE_NAME}] confirming task at {coordinates} for robot "
                f"'{self.bid_store[coordinates].bid.robot_id}'")
            self.confirm_pub.publish(Confirmation(Bid=self.bid_store[coordinates].bid,
                                                  robot_id=self.bid_store[coordinates].bid.robot_id))

        self.bid_store[coordinates].active = False


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    _no_of_robots_ = rospy.get_param('~no_of_robots', 2)
    _timeout_sec_ = rospy.get_param('~timeout_sec', 5)
    _refresh_robot_number_interval_ = rospy.get_param('~refresh_robot_number_interval', 60)

    TaskAllocator(_no_of_robots_, _timeout_sec_, _refresh_robot_number_interval_)
