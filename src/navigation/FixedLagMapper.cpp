#include "FixedLagMapper.h"

FixedLagMapper::FixedLagMapper(ros::NodeHandle& nh)
    :   lidar_buffer_(nh), 
        pose_graph_manager_(nh, lidar_buffer_),
        pose_graph_(pose_graph_manager_.graph()){
}

void FixedLagMapper::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    int pose_idx = pose_graph_manager_.imuCallback(msg);
    if (pose_idx != -1)
        newPose(pose_idx);
}

void FixedLagMapper::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    int pose_idx = pose_graph_manager_.gnssCallback(msg);
    if (pose_idx != -1)
        newPose(pose_idx);
}

void FixedLagMapper::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    lidar_buffer_.addPoints(msg);
}

void FixedLagMapper::newPose(int pose_idx){
    ROS_INFO_STREAM("New pose: " << pose_idx);
    // Add new frame to FixedLagFrameBuffer 
    
}