#include <ros/ros.h>
#include "navigation/PoseEstimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh;
    
    PoseEstimator pose_estimator(nh);

    ros::spin(); 
    return 0;
}