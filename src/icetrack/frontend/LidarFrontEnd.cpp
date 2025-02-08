#include "LidarFrontEnd.h"

LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    lidar_odometry_(nh, pose_graph, frame_buffer_){
}

void LidarFrontEnd::pollUpdates(){
    frame_buffer_.pollUpdates(); // Should be polled before lidar_odometry_ (as the odometry might use the frames)
    surface_estimator_.pollUpdates();
    lidar_odometry_.pollUpdates();
}

void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    pollUpdates();
}