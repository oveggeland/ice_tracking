#include "icetrack/mapping/icemap.h"


IceMap::IceMap(){
    ROS_INFO("HEI");
}

void IceMap::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("PCL CALLBACK");
}

void IceMap::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    ROS_INFO("POSE CALLBACK");
}