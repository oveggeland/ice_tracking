#include "LidarFrontEnd.h"

LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, point_buffer_),
    lidar_odometry_(nh, pose_graph, frame_buffer_){

    // Setup subscribers
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_sub_ = nh.subscribe(pose_topic, 1, &LidarFrontEnd::poseCallback, this);
}


/*
NB: To allow other ways of triggering this
When new state is available:
- Generate a new frame
- Perform frame-to-frame odometry
*/
void LidarFrontEnd::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    int state_idx = pose_graph_.getCurrentStateIdx();

    frame_buffer_.pollUpdates(); // Should be polled before lidar_odometry_ (as the odometry might use the frames)
    lidar_odometry_.pollUpdates();

    state_idx_ = state_idx;
}


void LidarFrontEnd::surfaceEstimation(){
    GraphNode state = pose_graph_.getCurrentState();

    // First check if we have tried this before
    if (surface_estimator_.getTimeStamp() == state.ts)
        return;

    // Second, check if we should wait a bit more
    if (pose_graph_.isInit() && !surface_estimator_.ready(state.ts))
        return;

    // Awesome, try to fit a pose and see
    if (surface_estimator_.estimateSurface(state.ts)){
        pose_graph_.surfaceCallback(state.idx, surface_estimator_.getPlaneCoeffs());
    }
}


/*
When new points arrive, add to the point buffer. Then:
- Signal surface_estimator about a new pcl event, potentially triggering a surface fit.
*/
void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    
    surfaceEstimation();
}