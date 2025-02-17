#include "LidarFrontEnd.h"

LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    lidar_odometry_(nh, pose_graph, frame_buffer_){
    
    double poll_interval = 0.1;
    timer_ = nh.createTimer(ros::Duration(poll_interval), &LidarFrontEnd::eventPoller, this);
}

void LidarFrontEnd::eventPoller(const ros::TimerEvent&) {
    int state_idx = pose_graph_.getCurrentStateIdx();
    if (state_idx > state_idx_)
        newStateEvent(state_idx);
}

/*
When new points arrive, add to the point buffer. Then:
- Signal surface_estimator about a new pcl event, potentially triggering a surface fit.
*/
void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    surface_estimator_.pollUpdates();
}


/*
When new state is available:
- Generate a new frame
- Perform frame-to-frame odometry
*/
void LidarFrontEnd::newStateEvent(int state_idx){
    ROS_INFO_STREAM("New state " << state_idx);
    frame_buffer_.pollUpdates(); // Should be polled before lidar_odometry_ (as the odometry might use the frames)
    lidar_odometry_.pollUpdates();

    state_idx_ = state_idx;
}