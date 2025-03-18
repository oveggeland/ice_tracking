#include <ros/ros.h>
#include "fixed_lag_mapping/FixedLagMapper.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;
    
    FixedLagMapper fixed_lag_mapper(nh);

    ros::spin(); 
    return 0;
}