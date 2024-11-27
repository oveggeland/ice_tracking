/*
Entry point for navigation node. This node subscribes to topics providing navigation measurements and publish a pose.
The states and covariances can optionally be written to an output csv file.
*/

#include "ros/ros.h"

#include "icetrack/navigation/icenav.h"

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;

    std::vector<std::string> topics;
    if (!nh.getParam("/topics", topics)){
        ROS_ERROR("Failed to retrieve topic names from parameter server.");
        return 1;
    }

    // Initialize some navigation node
    IceNav nav_object = IceNav();

    ros::Subscriber imu_sub = nh.subscribe(topics[0], 1000, &IceNav::imuCallback, &nav_object);
    ros::Subscriber gnss_sub = nh.subscribe(topics[1], 100, &IceNav::gnssCallback, &nav_object);
    ros::Subscriber pcl_sub = nh.subscribe(topics[2], 100, &IceNav::pclCallback, &nav_object);

    ros::spin();
    return 0;
}