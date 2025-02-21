#include "frontend/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    odometry_estimator_(nh, pose_graph, frame_buffer_),
    cloud_processor_(nh),
    cloud_publisher_(nh) {

    // Config
    getParamOrThrow<bool>(nh, "/cloud_manager/publish_frames", publish_frames_);
    getParamOrThrow<bool>(nh, "/cloud_manager/publish_processed", publish_processed_);

    // Setup subscribers
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_sub_ = nh.subscribe(pose_topic, 1, &CloudManager::poseCallback, this);
}


/*
On new state events, we do the following steps:
1. Maintain frame buffer (discard old frames, refine frames)
2. Create a new frame
3. Estimate odometry (frame-to-frame)
4. Process refined cloud
5. Publish refined cloud
*/
void CloudManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    int state_idx = pose_graph_.getCurrentStateIdx();

    // Maintain frame buffer
    frame_buffer_.removeOldFrames();
    frame_buffer_.refineFrames();

    // Create a new frame, return true on success
    if (frame_buffer_.createFrame(state_idx)){
        // Odometry with new frame
        // odometry_estimator_.estimateOdometry(state_idx); // TODO: Optimize odometry
        
        // Publish raw frame?
        if (publish_frames_){
            auto frame = frame_buffer_.back();
            cloud_publisher_.publishRawCloud(frame.global()->points_, frame.intensities());
        }
    }

    // Update map
    raw_cloud_ = frame_buffer_.buildMap();
    processed_ = false;

    // Publish
    if (publish_processed_){
        processed_cloud_ = cloud_processor_.processCloud(raw_cloud_);
        processed_ = true;

        cloud_publisher_.publishProcessedCloud(processed_cloud_);
    }
}

/*
On new lidar measurements, we do the following steps:
1. Add points to buffer
2. Poll surface estimator
*/
void CloudManager::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    point_buffer_.addPoints(msg);
    surface_estimator_.surfaceEstimation();
}