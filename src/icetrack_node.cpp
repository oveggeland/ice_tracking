#include <ros/ros.h>
#include "FixedLagMapper.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icetrack_node");
    ros::NodeHandle nh;
    
    FixedLagMapper fixed_lag_mapper(nh);

    ros::spin(); 
    return 0;
}