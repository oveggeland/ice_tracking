#include <ros/ros.h>
#include "navigation/FixedLagMapperInterface.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    
    FixedLagMapperInterface fixed_lag_mapper_interface(nh);

    ros::spin(); 
    return 0;
}