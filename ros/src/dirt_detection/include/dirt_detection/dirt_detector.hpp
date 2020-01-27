#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include "goal_manager_msgs/GoalObject.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/tfMessage.h>
#include <map>
#include <cmath>
#include <random>
#include "knowledge_aggregator_msgs/SO.h"
#include "knowledge_aggregator_msgs/DoAggregation.h"
#include "knowledge_aggregator_msgs/PartialObservation.h"
#include "robot_pose_publisher/RobotLocation.h"
#include "robot_pose_publisher/RobotLocations.h"

class DirtDetector
{
    //For node handling
    std::string node_name_;;
    ros::NodeHandle node_;

    //For receiving data from the parameter server 
    std::string robot_specifier;
    std::string sourceFrame;

    bool false_positive;
    int false_positive_prob;

    //Topics to be Subscribed to
    std::string map_topic_ = "map"; //"/robot_0/map";
    std::string scan_topic_ = "scan";
    std::string robot_locations_topic = "robots_locations";

    std::string partial_observation_svc = "partial_observation";

    //Topic to publish at
    std::string detected_dirt_topic_ = "detected_dirt";

    //Subscribers
    ros::Subscriber map_subscriber;
    ros::Subscriber scan_subscriber;

    ros::Subscriber locations_subscriber;

    ros::ServiceClient doAggregation;
    ros::ServiceClient getLocations;

    //Publisher
    ros::Publisher detected_dirt_publisher;
    ros::Publisher partial_observation_publisher;

    //For publishing in a format that's recognizable by the GoalList Node
    goal_manager_msgs::GoalObject detectedDirt;

    //knowledge_aggregator::SubjectiveOpinionList so;
	std::vector<knowledge_aggregator_msgs::SO> partialObservation;

    //For getting the occupancy map
    nav_msgs::OccupancyGrid map;
    // For getting the laser scan
    sensor_msgs::LaserScan scan;

    geometry_msgs::Pose r_pose;

    std::vector<robot_pose_publisher::RobotLocation> robot_locations;
    std::vector<geometry_msgs::Point> obstacles;

    //Laser Point Cloud for transforming the values gotten through the laser scan
    //into points on the map
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
    sensor_msgs::PointCloud cloud;

    bool useSubjectiveLogic;

private:
    // For finding the dirt and publishing it
    void DetectDirt();

public:

    void locationsCallBack(const robot_pose_publisher::RobotLocations::ConstPtr &msg);
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    float GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinate);
    float GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinate);
    DirtDetector();
    ~DirtDetector();
    void SubscribeToTopics();
    void PublishDetectedDirt();
    void PublishPartialObservation();
    void spin();

    knowledge_aggregator_msgs::SO getSubjectiveOpinion(geometry_msgs::PoseStamped, bool isDirt);

    int get_cell_index(float x, float y);
    bool is_occupied_(float x, float y);
    void findObstaclesInMap(void);
    bool isRobotInLocation(float x, float y);
    std::string getRobotId(void);

    bool genFalsePositive(void);
    //std::vector<std::tuple<float, float>> getPointsInLine(float x1, float y1, float x2, float y2);
};
