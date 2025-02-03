#include <ros/ros.h>
#include "navigation/FixedLagMapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    
    FixedLagMapper fixed_lag_mapper(nh);

    ros::spin(); 
    return 0;
}