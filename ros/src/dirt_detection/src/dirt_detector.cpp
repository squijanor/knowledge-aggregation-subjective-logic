#include "dirt_detection/dirt_detector.hpp"

//Global parameter to enable/disable the dirt detection
//(disabling needed e.g. if the dirt generation should publishes directly the dirt positions and together with an active detection duplicates would be created on topic /detected_dirt)
const int GLOBAL_ENABLE_DETECTION = true;

//Global parameter for the initial trust value which is used for a detected dirt (should be between 0 and 100, where 0 is not reliable dirt and 100 is completely reliable dirt)
const int GLOBAL_TRUST_VALUE = 100;

//Global variable for ID of next detected dirt
int global_id = 1;

//The Constructor
DirtDetector::DirtDetector() : laser_sub(node_, sourceFrame, 10), laser_notifier(laser_sub, listener, "map", 100)
{

    //A private node handle for getting data from the parameter server
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();

    //Getting which robot the node is being executed for
    private_nh.getParam("robot_specifier", robot_specifier);

    if (!private_nh.getParam("false_positive", false_positive))
    {
        false_positive = false;
    }

    if (!private_nh.getParam("false_positive_prob", false_positive_prob)){
        false_positive_prob = 0;
    }

    //Setting the value of the source frame here which is used by 'laser_sub' to transform the data to the map frame
    sourceFrame = robot_specifier + "/base_scan";

    detected_dirt_topic_ = "/detected_dirt";
    partial_observation_svc = "/partial_observation";
    robot_locations_topic = "/robots_locations";

    // ros::service::waitForService(partial_observation_svc);

    useSubjectiveLogic = true;
    if (!node_.getParam("/use_subjective_logic", useSubjectiveLogic))
    {
        useSubjectiveLogic = false;
    }
}

//The Destructor
DirtDetector::~DirtDetector()
{
}

//For Subscribing to all the necessary topics
void DirtDetector::SubscribeToTopics()
{
    //For Subscribing to "map"
    if (!map_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), map_topic_.c_str());
        map_subscriber = node_.subscribe(map_topic_, 1, &DirtDetector::mapCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), map_topic_.c_str());
    }

    //For Subscribing to "scan"
    if (!scan_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), scan_topic_.c_str());
        scan_subscriber = node_.subscribe(scan_topic_, 1, &DirtDetector::scanCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), "scan_topic_");
    }

    // doAggregation = node_.serviceClient<knowledge_aggregator_msgs::DoAggregation>(partial_observation_svc, true);

    if (!robot_locations_topic.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), robot_locations_topic.c_str());
        locations_subscriber = node_.subscribe(robot_locations_topic, 10, &DirtDetector::locationsCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), robot_locations_topic.c_str());
    }
}

//For publishing to the 'detected_dirt' topic
void DirtDetector::PublishDetectedDirt()
{
    ROS_INFO("[%s]: Publisher of topic '%s'", node_name_.c_str(), detected_dirt_topic_.c_str());
    detected_dirt_publisher = node_.advertise<goal_manager_msgs::GoalObject>(detected_dirt_topic_, 100);
}

void DirtDetector::PublishPartialObservation(){
    ROS_INFO("[%s]: Publisher of topic '%s'", node_name_.c_str(), partial_observation_svc.c_str());
    partial_observation_publisher =
        node_.advertise<knowledge_aggregator_msgs::PartialObservation>(partial_observation_svc, 100);
}

void DirtDetector::locationsCallBack(const robot_pose_publisher::RobotLocations::ConstPtr &msg)
{
    robot_locations = msg->locations;

    //ROS_INFO("Robot specifier: %s", robot_specifier.c_str());
    for (int i = 0; i < robot_locations.size(); i++){
        if (robot_locations[i].robot_id.compare(robot_specifier) == 0){
            r_pose = robot_locations[i].pose;
        }
    }
}

//The call back for subsciption to 'map'
void DirtDetector::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO("[%s]: executing map callback", robot_specifier.c_str());

    map = *msg;

    findObstaclesInMap();
}

//The call back for subsciption to 'scan'
void DirtDetector::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;

    /*try
    {
        //Used to transfrom data from the scan topic to points in the global map
        ros::Duration(1).sleep();
	    //ros::Duration().fromSec(msg->ranges.size()*msg->time_increment).sleep();
        projector.transformLaserScanToPointCloud("map", *msg, cloud, listener, laser_geometry::channel_option::Distance);
    }
    catch (tf::TransformException &e)
    {
        ROS_INFO("[%s]: wait for transform failed:\n%s", robot_specifier.c_str(), e.what());
        std::cout << e.what();
        return;
    }*/

    if(!listener.waitForTransform(msg->header.frame_id,
                                    "map",
                                    msg->header.stamp +
                                        ros::Duration().fromSec(
                                            msg->ranges.size()*msg->time_increment),
                                    ros::Duration(1.0))){
        ROS_INFO("[%s]: wait for transform failed", robot_specifier.c_str());
        return;
    }

    projector.transformLaserScanToPointCloud("map", *msg, cloud, listener, laser_geometry::channel_option::Distance);

    DetectDirt();
}

int DirtDetector::get_cell_index(float x, float y){
    int cell_x = int((x - map.info.origin.position.x) / map.info.resolution);
    int cell_y = int((y - map.info.origin.position.y) / map.info.resolution);

    int index = cell_x + cell_y * map.info.width;

    return index;
}

bool DirtDetector::is_occupied_(float x, float y)
{
    int index = get_cell_index(x, y);
    return map.data[index] != 0;
}

void DirtDetector::findObstaclesInMap(void){
    float X_MIN_IN = -5.0;
    float X_MAX_IN = 5.0;
    float Y_MIN_IN = -5.0;
    float Y_MAX_IN = 5.0;
    float x_min = map.info.origin.position.x;
    float y_min = map.info.origin.position.y;
    float x_max = x_min + map.info.height * map.info.resolution;
    float y_max = y_min + map.info.width * map.info.resolution;
    float x_step = map.info.resolution;
    float y_step = map.info.resolution;
    geometry_msgs::Point point;

    // Take always the center position of the grid cells
    for (float x = x_min + x_step; x < x_max - x_step; x+=x_step)
    {
        // Take always the center position of the grid cells
        for (float y = y_min + y_step; y < y_max - y_step; y+=y_step)
        {
            // Check if it is inside the movement area of the robots
            if ((x >= X_MIN_IN && x <= X_MAX_IN) && (y >= Y_MIN_IN && y <= Y_MAX_IN))
            {
                //int index = get_cell_index(x, y);
                // ROS_INFO("Obstacle at (%f,%f)", (float)x, (float)y);
                if (is_occupied_(x, y)){
                    point.x = x;
                    point.y = y;
                    point.z = 0.0;
                    obstacles.push_back(point);
                }
            }
        }
    }
}

bool DirtDetector::isRobotInLocation(float x, float y){
    float robot_size = 0.105*2;
    float robot_radius = robot_size / 2;
    bool isRobot = false;

    for (int i = 0; i < robot_locations.size(); i++){
        float r_x = robot_locations[i].pose.position.x;
        float r_y = robot_locations[i].pose.position.y;
        bool not_in_x_axis = false;
        bool not_in_y_axis = false;

        not_in_x_axis = x < (r_x - robot_radius) && x > (r_x + robot_radius);
        not_in_y_axis = y < (r_y - robot_radius) && y > (r_y + robot_radius);

        if (!(not_in_x_axis && not_in_y_axis)){
            isRobot = true;
            break;
        }
    }

    return isRobot;
}

bool DirtDetector::genFalsePositive(void){
    bool fp = false;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1,100);
    auto generate = std::bind ( distribution, generator );

    int fp_prob = rand() % 100;

    //ROS_INFO("FP prob %d", fp_prob);
    if (fp_prob < false_positive_prob)
    {
        fp = true;
    }

    return fp;
}

//Where the magic happends. Function for detecting dirt and publishing it
void DirtDetector::DetectDirt()
{
    geometry_msgs::PoseStamped pose_stamped_laser_map;
    std::vector<geometry_msgs::Pose> dirtCoordinates;
    std::vector<geometry_msgs::Pose> opinionCoordinates;

    knowledge_aggregator_msgs::PartialObservation opinions;
    opinions.partial_observation.clear();
    std::map<int, knowledge_aggregator_msgs::SO> opinion_cloud;

    nav_msgs::OccupancyGrid map_;
    map_ = map;

    //Iterating over the coordinates received from the transformation of the scan data to the point cloud data
    for (int i = 0; i < cloud.points.size(); i++)
    {
        // The cel number occupied by the coordinates of the obstacle received
        double cell_number = 0;

        // Variable to check whether a dirt patch has been detected or not
        bool is_dirt = false;

        knowledge_aggregator_msgs::SO so;

        // Varaibles for setting how many cells adjacent to the obstacle will be checked along the x and y axes
        int cell_number_iter_min_x = 3;
        int cell_number_iter_max_x = 3;
        int cell_number_iter_min_y = cell_number_iter_min_x;
        int cell_number_iter_max_y = cell_number_iter_max_x;

        //Variables to keep track of how many of the cells being checked are free and how many are not in orde to distinguish the
        // obstacle from the static obstacles such as walls around
        int cellCountFree_X = 0;
        int cellCountOccupied_X = 0;
        int cellCountFree_Y = 0;
        int cellCountOccupied_Y = 0;

        // Variables for tuning how many of the free and occupied cells should determine whether the dtected object is a patch of dirt or not
        int cell_count_check_free_x = 6;
        int cell_count_check_occupied_x = 0;
        int cell_count_check_free_y = cell_count_check_free_x;
        int cell_count_check_occupied_y = cell_count_check_occupied_x;

        //Getting the coordinates from the point cloud
        pose_stamped_laser_map.pose.position.x = cloud.points[i].x;
        pose_stamped_laser_map.pose.position.y = cloud.points[i].y;
        pose_stamped_laser_map.pose.position.z = cloud.points[i].z;

         //Calculating the distance of the coordinates from the origin as that is the real world pose of (0,0) in the occupancy grid
        float distance_from_origin_x = cloud.points[i].x - map.info.origin.position.x;
        float distance_from_origin_y = cloud.points[i].y - map.info.origin.position.y;

        //Based on the above calculated distance getting the values that would then be used to calculate the cell number
        int cell_num_x = distance_from_origin_x / map.info.resolution;
        int cell_num_y = distance_from_origin_y / map.info.resolution;

        //Calculating the cell number keeping in mind that the occupancy grid is a row major ordered array
        cell_number = cell_num_x + (cell_num_y * map.info.width);

        //Checking adjacent cells along the x axis
        for (int i = cell_number - cell_number_iter_min_x; i < cell_number + cell_number_iter_max_x; i++)
        {

            //ROS_INFO("[%s]: i=%d, 0 < i < %d ", robot_specifier.c_str(), i, map.info.width*map.info.height);

            if (i > map.info.width*map.info.height || i < 0)
                continue;
            //Getting the probability of a specific cell being occupied
            int occupancy_prob = map.data[i];
            if (occupancy_prob == 0)
            {
                cellCountFree_X = cellCountFree_X + 1;
            }

            if (occupancy_prob == 100)
            {
                cellCountOccupied_X = cellCountOccupied_X + 1;
            }
}

        //Checking adjacent cells along the y axis
        for (int i = cell_number - cell_number_iter_min_y * map.info.width; i < cell_number + cell_number_iter_max_y * map.info.width;)
        {
            if (i > map.info.width*map.info.height || i < 0)
                continue;

            int occupancy_prob = map.data[i];
            if (occupancy_prob == 0)
            {
                cellCountFree_Y = cellCountFree_Y + 1;
            }
            if (occupancy_prob == 100)
            {
                cellCountOccupied_Y = cellCountOccupied_Y + 1;
            }

            //Because the occupancy gird is a row major ordered array
            i = i + map.info.width;
        }

        // Determining whether what was detected was a patch of dirt or not
        if (cellCountFree_X >= cell_count_check_free_x && cellCountOccupied_X == cell_count_check_occupied_x && cellCountFree_Y >= cell_count_check_free_y && cellCountOccupied_Y == cell_count_check_occupied_y)
        {
            is_dirt = true;

            float tolerance = map.info.resolution;

            for (int i = 0; i < obstacles.size(); ++i)
            {
                if (std::abs(pose_stamped_laser_map.pose.position.x) - std::abs(obstacles[i].x) == 0 &&
                    std::abs(pose_stamped_laser_map.pose.position.y) - std::abs(obstacles[i].y) == 0)
                {
                    //ROS_INFO("Obstacle detected (%f,%f)", (float)obstacles[i].x, (float)obstacles[i].y);
                    is_dirt = false;
                    break;
                }
            }

/*
            //Check for whether what was detected was the other robot or not
            if (!isRobotInLocation(pose_stamped_laser_map.pose.position.x, pose_stamped_laser_map.pose.position.y)){
                float tolerance = map.info.resolution;


                for (int i = 0; i < obstacles.size(); ++i)
                {
                    if (std::abs(pose_stamped_laser_map.pose.position.x - obstacles[i].x) > tolerance &&
                        std::abs(pose_stamped_laser_map.pose.position.y - obstacles[i].y) > tolerance)
                    {
                        is_dirt = true;
                        break;
                    }
                }


                is_dirt = true;
            }
*/

        }

        bool fp = false;
        if (false_positive == true){
            fp = genFalsePositive();

            //ROS_INFO("[%s]: is_dirt=%d, fp=%d ", "FP", is_dirt, fp);
            if (is_dirt == false && fp == true)
                is_dirt == true;
        }

        //What to do if this is a dirt
        if (is_dirt == true)
        {
            //Adding the coordinates to a vector
            dirtCoordinates.push_back(pose_stamped_laser_map.pose);
            //Gettting the mid point of the maximum and minimum values for the obstacle along each axis
            pose_stamped_laser_map.pose.position.x = GetDirtX(dirtCoordinates);
            pose_stamped_laser_map.pose.position.y = GetDirtY(dirtCoordinates);
            //Preparing the format in which it's supposed to be published
            detectedDirt.pose = pose_stamped_laser_map.pose;
            detectedDirt.id = global_id;
            global_id++;
            detectedDirt.trust_value = GLOBAL_TRUST_VALUE;
            if (fp == true)
                detectedDirt.fp = 1;
            else
                detectedDirt.fp = 0;
            // ROS_INFO("Robot : [%s], Object Found at at : x = %f, y = %f , \nCountFree_X [%d], CountOccupied_X [%d], \nCountFree_Y [%d], CountOccupied_Y [%d]", robot_specifier.c_str(), detectedDirt.pose.position.x, detectedDirt.pose.position.y, cellCountFree_X, cellCountOccupied_X, cellCountFree_Y, cellCountOccupied_Y);
            detected_dirt_publisher.publish(detectedDirt);
        }

        // Create a subjective opinion for the current Pose
		if (useSubjectiveLogic == true)
		{
		    
            for (int i = 0; i < robot_locations.size(); i++){
                if (robot_locations[i].robot_id.compare(robot_specifier) == 0){
                    r_pose = robot_locations[i].pose;
	                break;
                }
            }
		    opinionCoordinates.clear();
		    opinionCoordinates.push_back(pose_stamped_laser_map.pose);
		    pose_stamped_laser_map.pose.position.x = GetDirtX(opinionCoordinates);
		    pose_stamped_laser_map.pose.position.y = GetDirtY(opinionCoordinates);
		    if (opinion_cloud.find(cell_number) == opinion_cloud.end()){
		        so = getSubjectiveOpinion(pose_stamped_laser_map, is_dirt);
		        opinion_cloud[cell_number] = so;
		        opinions.partial_observation.push_back(so);
		    }

		    //opinionCoordinates.clear();
		}
    }

    if (useSubjectiveLogic == true)
    {
        if (opinions.partial_observation.size() > 0){
            //doAggregation.call(opinions);
            partial_observation_publisher.publish(opinions);
            opinion_cloud.clear();
        }
    }
}

/*!
 * Generates a new subjective opinion for the given Pose
 */
knowledge_aggregator_msgs::SO DirtDetector::getSubjectiveOpinion(geometry_msgs::PoseStamped pose_stamped_laser_map,
		bool isDirt) {

	float x_tgt = pose_stamped_laser_map.pose.position.x;
	float y_tgt = pose_stamped_laser_map.pose.position.y;
	float x_src = r_pose.position.x;
	float y_src = r_pose.position.y;

	float unit = 1.0;
	float belief = 0.0;
	float disbelief = 0.0;
	float uncertainty = 0.0;
	float atomicity = 0.5;
	//float distance = map.info.resolution * std::hypot((x_tgt - x_src), (y_tgt - y_src));
	float distance = std::hypot((x_tgt - x_src), (y_tgt - y_src));

	float u = 0.0;
        if (distance >= scan.range_max)
            u = 0.99;
        else
            u = distance / scan.range_max;

	uncertainty = std::min(unit, u);
	if (isDirt) {
		belief = unit - uncertainty;
		disbelief = 0.0;
	} else {
		disbelief = unit - uncertainty;
		belief = 0.0;
	}

	knowledge_aggregator_msgs::SO so;
	so.pose = pose_stamped_laser_map.pose;
	so.belief = belief;
	so.disbelief = disbelief;
	so.uncertainty = uncertainty;
	so.base_rate = atomicity;

	return so;
}

//Getting the mid point of the maximum and minimum values for the obstacle along the x axis
float DirtDetector::GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.x;
    float max = dirtCoordinates[0].position.x;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x < min)
        {
            min = dirtCoordinates[i].position.x;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x > max)
        {
            max = dirtCoordinates[i].position.x;
        }
    }
    return (min + max) / 2;
}

//Getting the mid point of the maximum and minimum values for the obstacle along the y axis
float DirtDetector::GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.y;
    float max = dirtCoordinates[0].position.y;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y < min)
        {
            min = dirtCoordinates[i].position.y;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y > max)
        {
            max = dirtCoordinates[i].position.y;
        }
    }
    return (min + max) / 2;
}

void DirtDetector::spin()
{
    /*ros::Rate r(0.5);
    while (ros::ok())
    {
        DetectDirt();
        ros::spinOnce();
        r.sleep();
    }*/

    ros::spin();
}

std::string DirtDetector::getRobotId(void){
    return robot_specifier;
}

int main(int argc, char **argv)
{
    if (GLOBAL_ENABLE_DETECTION == true)
    {
        ros::init(argc, argv, "goal_detector");
        DirtDetector DirtDetector;
        DirtDetector.SubscribeToTopics();
        DirtDetector.PublishDetectedDirt();
        DirtDetector.PublishPartialObservation();

        DirtDetector.spin();
    }
    return 0;
}
