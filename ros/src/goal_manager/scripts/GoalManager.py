#! /usr/bin/env python3

from datetime import datetime
import csv
import rospy
import rospkg
import numpy
from typing import List, Dict, NamedTuple
from scipy import spatial

from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from std_msgs.msg import Header

from goal_manager_msgs.msg import GoalObject, GoalObjectList, DirtModel
from commons_msgs.msg import Goal
from commons.OccupancyMap import OccupancyMap, Cell
from knowledge_aggregator_msgs.srv import GetUncertainty, GetUncertaintyRequest, GetUncertaintyResponse
from knowledge_aggregator_msgs.msg import SO, SOList, PartialObservation

try:
    import jpy
except ImportError as err:
    print(f"[WARN] Could not import numpy: {err} - Subjective Logic knowledge aggregation won't work!!!")

# Node name
NODE_NAME = 'goal_manager'

# Topics and services
# BASE_MAP_TOPIC = '/robot_0/map'
BASE_MAP_TOPIC = 'modified_occupancy_grid'
TF_TOPIC = 'tf'
GOALS_TOPIC = 'dirt_and_goals'
NEW_DIRT_TOPIC = 'new_dirt'
DETECTED_DIRT_TOPIC = 'detected_dirt'
SUCCEEDED_GOAL_TOPIC = 'goal_attained'
NEW_GOAL_TOPIC = 'new_goal'
GET_UNCERTAINTY_SVC = 'uncertainty_eprob'
CONFIRMATION_TOPIC = 'confirmation'

DIRT_MODEL_NAME_ASSIGNED = "dirt_object_detected.sdf"
DIRT_MODEL_NAME = "dirt_object_detected.sdf"
DIRT_MODEL_PACKAGE = "dirt_generator"

TRUST_THRESHOLD = 90
TRUST_INCREMENT = 10
TRUST_MIN = 0
TRUST_MAX = 100

BELIEF_THRESHOLD = 60

X_MIN_IN = -5.0
X_MAX_IN = 5.0
Y_MIN_IN = -5.0
Y_MAX_IN = 5.0

# #### KNOWLEDGE AGGREGATOR

PSEUDO_NODE_NAME = 'knowledge_aggregator'

# Topics and services
PARTIAL_OBSERVATIONS_SVC = 'partial_observation'
GET_UNCERTAINTY_PROB_SVC = 'uncertainty_eprob'
GOALS_TOPIC = 'dirt_and_goals'

SL_CLASSPATH = '-Djava.class.path=/home/hiwi01/sl/subjective-logic-java-library-0.1.0.jar'

aggregator = None

sl_oper = rospy.get_param(f'~subjective_logic_operator', "CBF")
sl_classpath = rospy.get_param(f'~sl_classpath', SL_CLASSPATH)


class Opinion(NamedTuple):
    belief: float
    disbelief: float
    uncertainty: float
    base_rate: float
    expected_probability: float


class KnowledgeAggregator:
    """
	Node responsible for the knowledge aggregation using Subjective Logic.

	When first launch, the node creates a grid (based in the world map) with vacuous opinions. Then,
	it will aggregate the information coming from the different robots in the world. The robots scan the world and
	create a partial observation map with subjective opinions, that later are received by this node.
	"""

    def __init__(self, sl_operator: str = "CBF", classpath: str = SL_CLASSPATH):
        self.sl_operator = sl_operator
        self.classpath = classpath
        self.opinion_map: Dict[int, Opinion] = dict()
        self.position_map: List[Point] = list()
        self.point_list = list()
        self.obstacles: List[Point] = list()
        self.SubjectiveOpinion = None
        self.ArrayList = None
        self.dirt_pos_tolerance: float = 0.25
        self.active_dirt_list: List[GoalObject] = list()

        self.get_opinion_srv = None
        self.do_aggregation_srv = None
        self.goals_sub = None

        self.__init_subjective_logic_library(self.classpath)

        self.__init_subscribers()
        self.__init_publishers()

        rospy.loginfo(f"[{PSEUDO_NODE_NAME}] node is ready - "
                      f"\n\tlistening for partial observation opinions on '{self.do_aggregation_srv.resolved_name}")
        # f"\n\tserving request on '{self.get_opinion_srv.resolved_name}")

        # self.occupancy_map = None
        self.occupancy_map = OccupancyMap.from_message(rospy.wait_for_message(BASE_MAP_TOPIC, OccupancyGrid))
        # self.dirt_pos_tolerance = self.occupancy_map.resolution
        self.kd_tree = None

    def __init_subjective_logic_library(self, classpath: str):
        jpy.create_jvm(['-Xmx512M', classpath])

        self.SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')
        self.ArrayList = jpy.get_type('java.util.ArrayList')

    def __init_subscribers(self):
        self.goals_sub = rospy.Subscriber(GOALS_TOPIC,
                                          GoalObjectList, self.__dirt_and_goals_list_cb)

        self.do_aggregation_srv = rospy.Subscriber(PARTIAL_OBSERVATIONS_SVC,
                                                   PartialObservation, self.do_aggregation_cb)

    def __init_publishers(self):
        self.get_opinion_srv = None

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

    def __get_closest_active_goal(self, point) -> bool:
        """
		Goes through the list with all currently active dirt positions and compare their positions with the given
		position. If the point is the proximity of an active goal, it returns such goal, None otherwise
		"""

        # Check all already published (active) dirt objects (stored and received from the goal_list)
        for dirt in list(self.active_dirt_list):
            if self.__comparing_points(point, dirt.pose.position):
                return dirt

        return None

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

        return self.occupancy_map.grid[index] >= 90

    def __is_occupied_(self, x, y) -> bool:
        index = self.__get_cell_index(x, y)

        return self.occupancy_map.grid[index] >= 90

    def __init_opinion_map(self):
        x_min = self.occupancy_map.origin.x
        y_min = self.occupancy_map.origin.y
        x_max = x_min + self.occupancy_map.height * self.occupancy_map.resolution
        y_max = y_min + self.occupancy_map.width * self.occupancy_map.resolution
        x_step = y_step = self.occupancy_map.resolution

        rospy.loginfo(rospy.get_caller_id() + "\n\n\t***Generating vacuous opinion map")
        # Take always the center position of the grid cells
        for x in numpy.arange(x_min + x_step / 2, x_max - x_step / 2, x_step):

            # Take always the center position of the grid cells
            for y in numpy.arange(y_min + y_step / 2, y_max - y_step / 2, y_step):

                # Check if it is inside the movement area of the robots
                if (X_MIN_IN <= x <= X_MAX_IN) and (Y_MIN_IN <= y <= Y_MAX_IN):

                    if not self.__is_occupied_(x, y):
                        self.position_map.append(Point(x=x, y=y, z=0.0))
                    else:
                        self.obstacles.append(Point(x=x, y=y, z=0.0))

        rospy.loginfo(rospy.get_caller_id() + "\n\n\t***There are %d free cells" % len(self.position_map))

        self.point_list = []
        for point in self.position_map:
            self.point_list.append((point.x, point.y))

            index = self.__get_cell_index(point.x, point.y)

            # vacuous = subjectiveOpinion.newVacuousOpinion(0.5)
            vacuous = self.SubjectiveOpinion(0.0, 0.0, 1.0, 0.5)
            opinion = Opinion(belief=vacuous.getBelief(),
                              disbelief=vacuous.getDisbelief(),
                              uncertainty=vacuous.getUncertainty(),
                              base_rate=vacuous.getBaseRate(),
                              expected_probability=vacuous.getExpectation())
            self.opinion_map[index] = opinion

        self.kd_tree = spatial.KDTree(self.point_list)

    def find_closest_cell(self, position):
        index = 0

        query = self.kd_tree.query([(position.x, position.y)])
        point_in_list = self.point_list[query[1][0]]

        for point in self.position_map:
            if point.x == point_in_list[0] and point.y == point_in_list[1]:
                index = self.__get_cell_index(point.x, point.y)
                break

        return index

    def do_aggregation_cb(self, request):

        # rospy.loginfo(f"[{NODE_NAME}] got {len(request.partial_observation)} opinions. Aggregating knowledge with "
        #               f"operator {self.sl_operator} ...")

        self.__init_subjective_logic_library(self.classpath)

        so_collection = self.ArrayList()

        for so in request.partial_observation:

            opinion_found = False
            index = self.__get_cell_index(so.pose.position.x, so.pose.position.y)
            index2 = self.find_closest_cell(so.pose.position)

            new_subjective_opinion = None
            subjective_opinion_in_map = None
            try:
                vacuous = self.SubjectiveOpinion(0.0, 0.0, 1.0, 0.5)
                opinion = Opinion(belief=vacuous.getBelief(),
                                  disbelief=vacuous.getDisbelief(),
                                  uncertainty=vacuous.getUncertainty(),
                                  base_rate=vacuous.getBaseRate(),
                                  expected_probability=vacuous.getExpectation())

                opinion = self.opinion_map[index2]
                subjective_opinion_in_map = self.SubjectiveOpinion(opinion.belief, opinion.disbelief,
                                                                   opinion.uncertainty, opinion.base_rate)

                opinion_found = True

            except KeyError:
                # rospy.loginfo(rospy.get_caller_id() + f"\n\n\t***Opinion at index ({so.pose.position.x},"
                #                                       f"{so.pose.position.y}={index2}) not found in map")
                opinion_found = False
                pass

            if opinion_found:
                new_subjective_opinion = self.SubjectiveOpinion(so.belief, so.disbelief, so.uncertainty, so.base_rate)

                so_collection.add(subjective_opinion_in_map)
                so_collection.add(new_subjective_opinion)

                aggregated_opinion = None
                if self.sl_operator == "CBF":
                    aggregated_opinion = self.SubjectiveOpinion.cumulativeCollectionFuse(so_collection)
                elif self.sl_operator == "CCF":
                    aggregated_opinion = self.SubjectiveOpinion.ccCollectionFuse(so_collection)
                elif self.sl_operator == "WBF":
                    aggregated_opinion = self.SubjectiveOpinion.weightedCollectionFuse(so_collection)
                elif self.sl_operator == "AVG":
                    aggregated_opinion = self.SubjectiveOpinion.average(so_collection)

                opinion = Opinion(belief=aggregated_opinion.getBelief(),
                                  disbelief=aggregated_opinion.getDisbelief(),
                                  uncertainty=aggregated_opinion.getUncertainty(),
                                  base_rate=aggregated_opinion.getBaseRate(),
                                  expected_probability=aggregated_opinion.getExpectation())
                self.opinion_map[index2] = opinion

                # rospy.loginfo(
                #     f"[{NODE_NAME}] ({so.pose.position.x},{so.pose.position.y}={index}) "
                #     f"New:({new_subjective_opinion.getBelief()},{new_subjective_opinion.getDisbelief()},"
                #     f"{new_subjective_opinion.getUncertainty()},{new_subjective_opinion.getExpectation()})"
                #     f"In Map:({subjective_opinion_in_map.getBelief()},{subjective_opinion_in_map.getDisbelief()},"
                #     f"{subjective_opinion_in_map.getUncertainty()},{subjective_opinion_in_map.getExpectation()})"
                #     f"Aggregated:({aggregated_opinion.getBelief()},{aggregated_opinion.getDisbelief()},"
                #     f"{aggregated_opinion.getUncertainty()},{aggregated_opinion.getExpectation()})")

                so_collection.clear()

        # return DoAggregationResponse()

    def get_uncertainty_cb(self, request: GetUncertaintyRequest) -> GetUncertaintyResponse:

        self.SubjectiveOpinion = jpy.get_type('de.tum.i4.subjectivelogic.SubjectiveOpinion')
        index = self.__get_cell_index(request.pose.position.x, request.pose.position.y)
        index2 = self.find_closest_cell(request.pose.position)

        vacuous = self.SubjectiveOpinion(0.0, 0.0, 1.0, 0.5)
        opinion = Opinion(belief=vacuous.getBelief(),
                          disbelief=vacuous.getDisbelief(),
                          uncertainty=vacuous.getUncertainty(),
                          base_rate=vacuous.getBaseRate(),
                          expected_probability=vacuous.getExpectation())
        try:
            opinion = self.opinion_map[index2]

        except KeyError:
            rospy.loginfo(f"[{NODE_NAME}] cell not found, sending a vacuous opinion")
            pass

        return GetUncertaintyResponse(belief=opinion.belief, uncertainty=opinion.uncertainty,
                                      expected_probability=opinion.expected_probability)

    def start_aggregator(self):
        self.__init_opinion_map()


class GoalManager:
    """
	Node responsible of managing the goals (i.e. detected dirt piles). The management consists of keeping track of
	goals and when they are attained (dirt is cleaned), publishing new goals to the Task Allocator and communicating
	with the Knowledge aggregator
	"""

    def __init__(self, sl_threshold):
        self.dirt_pos_tolerance: float = 0.25
        self.first_run: bool = True
        self.robot_size: float = 0.105 * 2
        self.robots_pose: List[Pose] = list()
        self.goal_list: List[GoalObject] = list()
        self.dirt_list: List[GoalObject] = list()
        self.dirt_and_goals: List[GoalObject] = list()
        self.dirt_undetected: List[DirtModel] = list()
        self.dirt_detected: List[DirtModel] = list()
        self.use_subjective_logic = rospy.get_param(f'/use_subjective_logic', False)
        self.position_map: List[Point] = []

        self.goal_attained: List[GoalObject] = list()
        self.spawned: List[GoalObject] = list()
        self.goals_pub = None
        self.detected_dirt_sub = None
        self.succeeded_goal_sub = None
        self.new_dirt_sub = None

        self.sl_threshold = sl_threshold

        self.occupancy_map = None

        self.aggregator = None

        self.__init_subscribers()
        self.__init_publishers()

        rospy.loginfo(f"[{NODE_NAME}] node is ready - "
                      f"\n\tlistening for new dirt spawned on '{self.new_dirt_sub.resolved_name}"
                      f"\n\tlistening for dirt detection on '{self.detected_dirt_sub.resolved_name}"
                      f"\n\tlistening for goals attained on '{self.succeeded_goal_sub.resolved_name}"
                      f"\n\tpublishing list of goals to '{self.goals_pub.resolved_name}'"
                      f"\n\tpublishing new tasks to '{self.new_task_pub.resolved_name}'")

    def __init_subscribers(self):
        self.detected_dirt_sub = rospy.Subscriber(DETECTED_DIRT_TOPIC, GoalObject, self.__detected_dirt_cb)
        self.succeeded_goal_sub = rospy.Subscriber(SUCCEEDED_GOAL_TOPIC, Goal, self.__goal_attained)
        self.new_dirt_sub = rospy.Subscriber(NEW_DIRT_TOPIC, DirtModel, self.__new_dirt_cb)

    def __init_publishers(self):
        self.goals_pub = rospy.Publisher(GOALS_TOPIC, GoalObjectList, queue_size=100)
        self.new_task_pub = rospy.Publisher(NEW_GOAL_TOPIC, Goal, queue_size=100)

    def __compare_poses(self, pose1, pose2) -> bool:
        """
		Compares two Poses and return true if they are identical, that is, they have the same position (with some
		tolerance)

		:param pose1: Pose to compare
		:param pose2: Pose to compare
		:return: True if the Poses are identical
		"""

        return abs(pose1.position.x - pose2.position.x) <= self.dirt_pos_tolerance and abs(
            pose1.position.y - pose2.position.y) <= self.dirt_pos_tolerance

    def __compare_goals(self, goal1, goal2) -> bool:
        """
		Compares two GoalObject instances and return true if they are identical (same ID and/or same position)
		:param goal1: first GoalObject instance
		:param goal2: second GoalObject instance
		:return: true if both instances are identical
		"""

        return self.__compare_poses(goal1.pose, goal2.pose)

    def __get_cell_index(self, x, y) -> int:
        cell_x = int((x - self.occupancy_map.origin.x) / self.occupancy_map.resolution)
        cell_y = int((y - self.occupancy_map.origin.y) / self.occupancy_map.resolution)

        index = cell_x + cell_y * self.occupancy_map.width
        # index = cell_y + cell_x * self.occupancy_map.height

        return index

    def __is_occupied_(self, x, y) -> bool:

        index = self.__get_cell_index(x, y)

        return self.occupancy_map.grid[index] != 0

    def find_closest_cell(self, position):
        points = []
        new_position = None
        for point in self.position_map:
            points.append((point.x, point.y))

        tree = spatial.KDTree(points)
        query = tree.query([(position.x, position.y)])
        point_in_list = points[query[1][0]]

        for point in self.position_map:
            if point.x == point_in_list[0] and point.y == point_in_list[1]:
                new_position = point
                break

        # rospy.loginfo(f"Point ({position.x},{position.y}) "
        #               f"belongs to cell: {point_in_list}/{new_position}, index: {index}")

        return new_position

    def __delete_dirt_model(self, dirt_pose):
        # receive pose of a dirt which was successfully reached and the corresponding model should be deleted now go
        # through the list of all detected dirt models, compare their positions with the given one and when they are
        # identical, delete it

        rospy.loginfo(rospy.get_caller_id() + "\tTrying to delete a dirt object (%f, %f)..." %
                      (dirt_pose.position.x, dirt_pose.position.y))
        for dirt_model in list(self.dirt_detected):

            if self.__compare_poses(dirt_pose, dirt_model.pose):
                rospy.loginfo(rospy.get_caller_id() + "\tDeleting the current dirt object...")
                rospy.wait_for_service('gazebo/delete_model')
                delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

                d_name = str(dirt_model.name)
                if "_detected" not in dirt_model.name:
                    d_name = str(dirt_model.name) + "_detected"
                delete_model(d_name)
                # delete_model(str(dirt_model.name))

                if dirt_model in self.dirt_detected:
                    self.dirt_detected.remove(dirt_model)

    def __goal_attained(self, goal):
        # Handler if a goal was successfully reached and can be deleted from the goal list

        rospy.loginfo(rospy.get_caller_id() + f"\n\tGoal attained message received, looking for goal"
                                              f"in position {goal.x, goal.y} to be deleted...")

        goal_pose = Pose(position=Point(x=goal.x, y=goal.y, z=0.0),
                         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        succeeded_goal = GoalObject()

        for _goal in self.goal_list:
            if self.__compare_poses(_goal.pose, goal_pose):
                succeeded_goal = _goal
                break

        rospy.loginfo(
            rospy.get_caller_id() + '\tReceived: succeeded goal object with ID: %d, position (%f,%f) and trust value: '
                                    '%d' %
            (succeeded_goal.id, succeeded_goal.pose.position.x, succeeded_goal.pose.position.y,
             succeeded_goal.trust_value))

        # Updating the global goal list by only taking goals from the old list which have not same ID and not same
        # position as the succeeded goal
        self.goal_list[:] = [
            _goal for _goal in self.goal_list if not self.__compare_goals(_goal, succeeded_goal)]

        # Delete dirt model (if it was even spawned)
        self.__delete_dirt_model(succeeded_goal.pose)

        self.dirt_and_goals = list(self.goal_list)

        self.goals_pub.publish(self.dirt_and_goals)

    def __new_dirt_cb(self, dirt: DirtModel):

        self.dirt_undetected.append(dirt)

        rospy.loginfo(
            rospy.get_caller_id() + "\tNew dirt model was added to dirt_models_undetected list: Name %s and position "
                                    "(%f, %f)" %
            (dirt.name, dirt.pose.position.x, dirt.pose.position.y))

    def __change_dirt_model(self, dirt_pose: Pose) -> Pose:
        # receive pose of a dirt which was detected and the corresponding model should be deleted now (with
        # collision) and spawned as new model (which has no collision --> robot can move into it) go through the list
        # of all received dirt models, compare their positions with the given one and when they are identical,
        # delete it and spawn a new one

        for dirt_model in list(self.dirt_undetected):

            if self.__compare_poses(dirt_pose, dirt_model.pose):
                # Creating a new model at the same position without collision (and other color) and prepare the
                # spawning (before deleting it) Init service
                rospy.wait_for_service("gazebo/spawn_sdf_model")
                spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

                name = dirt_model.name + "_detected"
                rospack = rospkg.RosPack()
                pkg_path = rospack.get_path(DIRT_MODEL_PACKAGE)
                path = pkg_path + "/" + DIRT_MODEL_NAME
                with open(path, "r") as data:
                    model = data.read()
                robot = ""  # can be left empty
                pose = dirt_model.pose
                frame = ""  # empty or "world" or "map"

                # Preparing the deletion
                rospy.wait_for_service('gazebo/delete_model')
                delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

                # Delete old model
                delete_model(str(dirt_model.name))
                if dirt_model in self.dirt_undetected:
                    self.dirt_undetected.remove(dirt_model)

                # Spawn new one
                spawn_model(name, model, robot, pose, frame)

                # save name of model combined with position for future deletion (means: added into the new list)
                new_dirt_model = DirtModel(header=Header(stamp=rospy.get_rostime(), frame_id="map"), name=name,
                                           pose=pose)
                self.dirt_detected.append(new_dirt_model)

                return pose
            # else:
            #     rospy.loginfo(rospy.get_caller_id() + "\tModel not found (%f, %f)..." %
            #                   (dirt_pose.position.x, dirt_pose.position.y))

        return None

    def __detected_dirt_cb(self, detected_dirt):
        # Handler if new detected dirt was published (add it to one of the lists or update its trust value if it is
        # already in a list)

        # Searching for a dirt/goal object in dirt list and in goal list with same position (or ID, what is very
        # unlikely). If found, update its trust value, otherwise add it as new goal to a list
        found = False

        detected_dirt_aux = detected_dirt

        # Check with the Knowledge aggregator the amount of uncertainty/expected prob. to decide if the goal should
        # be triggered as a new task to start the bidding process
        if self.use_subjective_logic:
            trust_threshold = self.sl_threshold
        else:
            trust_threshold = TRUST_THRESHOLD
        trust_value = 100

        if self.use_subjective_logic:
            req = GetUncertaintyRequest()
            req.pose = Pose()
            req.pose = detected_dirt_aux.pose

            if self.aggregator is not None:
                resp = self.aggregator.get_uncertainty_cb(req)
                trust_value = int(resp.expected_probability * 100)

        # Check if detected point is already a pursued goal
        for goal in self.goal_list:
            if self.__compare_goals(goal, detected_dirt_aux):
                return

        if detected_dirt_aux.fp == 1:
            self.dirt_list.append(detected_dirt_aux)

        # Check if the received detected dirt is already in the dirt list. If it is, check the trust value to decide
        # if the dirt is promoted to a goal to pursue
        for goal in list(self.dirt_list):
            if self.__compare_goals(goal, detected_dirt_aux):

                # If found, update its trust value
                if not self.use_subjective_logic:
                    goal.trust_value = min(TRUST_MAX, goal.trust_value + TRUST_INCREMENT)
                else:
                    goal.trust_value = trust_value
                found = True

                # Trust value has changed, so we need to check if trust threshold is now reached
                if goal.trust_value >= trust_threshold:
                    # If yes, then goal has to be moved from dirt_list to goal_list
                    # self.goal_list.append(goal)

                    if goal in self.dirt_list:
                        self.dirt_list.remove(goal)

                    # Change the model of the dirt to a goal model (without collision) (if spawning is enabled -->
                    # dirt_models lists will be filled)
                    model_pose: Pose = self.__change_dirt_model(goal.pose)
                    if model_pose is not None:
                        # Send task to the bidder

                        is_virtual = False
                        if goal.fp == 1:
                            is_virtual = True

                        task = Goal(
                            header=Header(stamp=rospy.get_rostime(), frame_id="map"), x=model_pose.position.x,
                            y=model_pose.position.y, is_virtual=is_virtual)
                        # rospy.sleep(0.2)
                        self.new_task_pub.publish(task)

                        self.goal_list.append(
                            GoalObject(id=goal.id, pose=model_pose, trust_value=goal.trust_value, fp=goal.fp))

                break

        # If received dirt was not found in dirt list, check if its is found on the goal list. If dirt is found,
        # just update trust value
        if not found:
            for goal in self.goal_list:
                if self.__compare_goals(goal, detected_dirt_aux):

                    if not self.use_subjective_logic:
                        goal.trust_value = min(TRUST_MAX, goal.trust_value + TRUST_INCREMENT)
                    else:
                        goal.trust_value = trust_value
                    found = True
                    break

        # If the received dirt was not found in dirt list nor in goal list, check its trust value and decide if it is
        # added to the dirt list or if should be immediately promoted to a goal to pursue
        if not found:
            self.dirt_list.append(detected_dirt_aux)

        self.goals_pub.publish(self.goal_list)

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
                for x in numpy.arange(x_min + x_step / 2, x_max - x_step / 2, x_step):

                    # Take always the center position of the grid cells
                    for y in numpy.arange(y_min + y_step / 2, y_max - y_step / 2, y_step):

                        # Check if it is inside the movement area of the robots
                        if (X_MIN_IN <= x <= X_MAX_IN) and (Y_MIN_IN <= y <= Y_MAX_IN):

                            if not self.__is_occupied_(x, y):
                                self.position_map.append(Point(x=x, y=y, z=0.0))

                break

    def publish_so_info(self, _=None):
        timestamp = datetime.now()
        for goal in self.goal_list:
            req = GetUncertaintyRequest()
            req.pose = Pose()
            req.pose = goal.pose

            resp = self.aggregator.get_uncertainty_cb(req)  # resp = self.get_uncertainty(req)

            with open(self.so_csv, 'a') as csv_file:
                writer = csv.writer(csv_file)

                entry = [timestamp, goal.id, goal.fp, resp.belief, resp.disbelief, resp.uncertainty,
                         resp.expected_probability]

                writer.writerow(entry)

    def goal_manager(self):
        self.occupancy_map = OccupancyMap.from_message(rospy.wait_for_message(BASE_MAP_TOPIC, OccupancyGrid))

        self.__get_dirt_candidate_cells()
        # self.dirt_pos_tolerance = self.occupancy_map.resolution

        self.aggregator = KnowledgeAggregator(sl_oper, sl_classpath)
        self.aggregator.occupancy_map = self.occupancy_map
        while self.aggregator.occupancy_map is None:
            var = self.occupancy_map
        self.aggregator.start_aggregator()

        rospy.spin()


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)

    sl_oper = rospy.get_param(f'~subjective_logic_operator', "CBF")
    sl_classpath = rospy.get_param(f'~sl_classpath', SL_CLASSPATH)

    sl_threshold_ = rospy.get_param(f'~sl_threshold', 80)

    gm = GoalManager(sl_threshold_)
    gm.goal_manager()
