#include "LidarFrontEnd.h"

LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    odometry_estimator_(nh, pose_graph, frame_buffer_),
    cloud_processor_(nh),
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
    if (frame_buffer_.createFrame(state_idx)){}
        odometry_estimator_.estimateOdometry(state_idx);

    // Process cloud
    auto raw_cloud = frame_buffer_.getTensorCloud();
    auto processed_cloud = cloud_processor_.processCloud(raw_cloud);

    ROS_INFO_STREAM("raw cloud isze is: " << getCloudSize(raw_cloud));
    if (getCloudSize(raw_cloud) > 1.0e5){
        visualizeCloud(raw_cloud);
        visualizeCloud(processed_cloud);
        // Save cloud
        // cloud_processor_.saveCloud(processed_cloud);
    }
    // cloud_publisher_.publishProcessed();
    // cloud_publisher_.publishRaw();
    
    // Publish most recent cloud
    // cloud_publisher_.publishCloud();
}


/*
When new points arrive, add to the point buffer. Then:
1. Signal surface_estimator about a new pcl event, potentially triggering a surface fit.
*/
void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addPoints(msg);
    
    surface_estimator_.surfaceEstimation();
}