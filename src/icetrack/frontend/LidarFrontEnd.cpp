#include "LidarFrontEnd.h"

LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    odometry_estimator_(nh, pose_graph, frame_buffer_),
    cloud_publisher_(nh, frame_buffer_){

    // Setup subscribers
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_sub_ = nh.subscribe(pose_topic, 1, &LidarFrontEnd::poseCallback, this);
}


/*
New state is available!
*/
void LidarFrontEnd::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    int state_idx = pose_graph_.getCurrentStateIdx();

    // Maintain frame buffer
    frame_buffer_.removeOldFrames();
    frame_buffer_.refineFrames();

    // Create a new frame, if succesful, perform lidar odometry
    if (frame_buffer_.createFrame(state_idx))
        odometry_estimator_.estimateOdometry(state_idx);

    // Publish most recent cloud
    cloud_publisher_.publishCloud();
}


/*
When new points arrive, add to the point buffer. Then:
1. Signal surface_estimator about a new pcl event, potentially triggering a surface fit.
*/
void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    
    surface_estimator_.surfaceEstimation();
}