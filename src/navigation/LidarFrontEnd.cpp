#include "LidarFrontEnd.h"


LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph_, point_buffer_),
    surface_estimator_(nh, point_buffer_){
    // Initialize

}


void LidarFrontEnd::planeFitting(int state_idx){
    double ts = pose_graph_.getTimeStamp(state_idx);

    if (surface_estimator_.estimateSurface(state_idx, ts)){
        pose_graph_.planeFitCallback(state_idx, surface_estimator_.getPlaneCoeffs());
    }
}




/*
Reactive function. Check for updates in pose graph or point buffer that requires action. 
*/
void LidarFrontEnd::pollUpdates(){
    // Get current state of pose graph
    int state_idx = pose_graph_.getCurrentStateIdx();
    // Generate frames
    // if (frame_buffer_.getIndex() < state_idx)
    //     frame_buffer_.addFrame(state_idx);
    
    // Fit plane
    if (surface_estimator_.getPlaneIndex() < state_idx)
        planeFitting(state_idx);

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