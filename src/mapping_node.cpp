#include <ros/ros.h>
#include "mapping/CloudManager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    
    CloudManager cloud_manager(nh);

    ros::spin(); 
    return 0;
}