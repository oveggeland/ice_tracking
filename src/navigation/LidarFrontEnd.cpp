#include "LidarFrontEnd.h"


LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph_, point_buffer_),
    surface_estimator_(nh){
}


void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // Add new points to buffer
    point_buffer_.addPoints(msg);

    // Check if we need to generate a lidar frame
    frame_buffer_.generateFrames();


    pose_graph_.planeFitCallback(0, Eigen::Vector4d(0, 0, 0, -17));
}