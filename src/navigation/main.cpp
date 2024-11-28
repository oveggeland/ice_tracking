/*
Entry point for navigation node. This node subscribes to topics providing navigation measurements and publish a pose.
The states and covariances can optionally be written to an output csv file.
*/

#include "ros/ros.h"

#include "icetrack/message_sequencer.h"
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

    // Initialize message sequencer
    MessageSequencer sequencer(nh, 1.0);

    // Attach callbacks for the relevant topics
    sequencer.attachCallback<sensor_msgs::Imu>(
        topics[0], 
        1000, 
        std::bind(&IceNav::imuCallback, &nav_object, std::placeholders::_1)
    );
    sequencer.attachCallback<sensor_msgs::NavSatFix>(
        topics[1], 
        100, 
        std::bind(&IceNav::gnssCallback, &nav_object, std::placeholders::_1)
    );
    sequencer.attachCallback<sensor_msgs::PointCloud2>(
        topics[2], 
        100, 
        std::bind(&IceNav::pclCallback, &nav_object, std::placeholders::_1)
    );

    ros::spin();
    return 0;
}