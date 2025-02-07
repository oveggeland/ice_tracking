#include "LidarFrontEnd.h"


LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    point_buffer_(nh), 
    pose_graph_(pose_graph),
    surface_estimator_(nh, point_buffer_),
    frame_buffer_(nh, pose_graph_, point_buffer_),
    lidar_odometry_(nh, pose_graph_, frame_buffer_){
    // Initialize

}

void LidarFrontEnd::planeFitting(){
    // NB: Not thread safe to request ts and idx in different calls
    double ts = pose_graph_.getCurrentTimeStamp();
    int idx = pose_graph_.getCurrentStateIdx();

    if (surface_estimator_.fitSurface(ts))
        pose_graph_.planeFitCallback(idx, surface_estimator_.getPlaneCoeffs());
}


/*
Reactive function. Check for possible actions to take. 
*/
void LidarFrontEnd::pollUpdates(){
    // Generate frames
    frame_buffer_.generateFrames();

    // Frame alignment
    lidar_odometry_.alignFrames();

    // Fit plane
    planeFitting();

    // Odometry
    // if (state_idx_ > odom_idx_)
    //     doOdometry(state_idx_);
}


/*
New points arrived. Add to buffer and poll for actions.
*/
void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    pollUpdates();
}