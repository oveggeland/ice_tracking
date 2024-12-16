/*
Entry point for icetrack node. This node use icenav and icemap to continously and create maps of the surrounding ice.
*/

#include <ros/ros.h>

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "rostopic_node");
    ros::NodeHandle nh;

    ROS_WARN("Icetrack subscriber node is not implemented");
    
    return 0;
}