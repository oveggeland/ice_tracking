#include "frontend/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh, pose_graph, point_buffer_),
    surface_estimator_(nh, pose_graph, point_buffer_),
    odometry_estimator_(nh, pose_graph, frame_buffer_),
    cloud_publisher_(nh),
    floe_manager_(nh, pose_graph, frame_buffer_) {

    // Config
    getParamOrThrow(nh, "/cloud_manager/publish_frames", publish_frames_);
    getParamOrThrow(nh, "/cloud_manager/publish_cloud", publish_cloud_);
    getParamOrThrow(nh, "/cloud_manager/cloud_pub_interval", cloud_pub_interval_);

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
    double ts = msg->header.stamp.toSec();
    int state_idx = pose_graph_.getCurrentStateIdx();

    // Maintain frame buffer
    frame_buffer_.removeOldFrames();
    frame_buffer_.refineFrames();

    // Create new frame
    bool new_frame = frame_buffer_.createFrame(state_idx);

    // Update floe positions
    floe_manager_.updateFloes();
    floe_manager_.expandFloes();
    floe_manager_.discoverFloes();
    floe_manager_.mergeFloes();


    // if (floe_manager_.floeCount() > 0)
    //     floe_manager_.visualizeFloes();
    

    // Odometry with new frame
    // odometry_estimator_.estimateOdometry(state_idx);
    
    // // Publish raw frame?
    // if (publish_frames_){
    //     auto frame = frame_buffer_.back();
    //     cloud_publisher_.publishFrame(frame.local().points_, frame.intensities());
    // }


    // raw_cloud_ = frame_buffer_.buildMap();

    // Publishing
    // if (publish_cloud_ && (ts - ts_last_cloud_pub_ > cloud_pub_interval_)){
    //     const auto points = raw_cloud_.ToLegacy().points_;
    //     const std::vector<float> intensities(points.size());
    //     cloud_publisher_.publishCloud(points, intensities);

    //     ts_last_cloud_pub_ = ts;
    // }
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