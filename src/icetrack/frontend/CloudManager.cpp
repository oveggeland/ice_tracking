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
        odometry_estimator_.estimateOdometry(state_idx); // TODO: Optimize odometry
        
        // Publish raw frame?
        if (publish_frames_){
            auto frame = frame_buffer_.back();
            cloud_publisher_.publishRawCloud(frame.global()->points_, frame.intensities());
        }
    }

    // Update map
    raw_cloud_ = frame_buffer_.buildMap();

    // Publish
    if (publish_processed_){
        processed_cloud_ = cloud_processor_.processCloud(raw_cloud_);
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



open3d::t::geometry::PointCloud CloudManager::cloudQuery(bool process, std::optional<double> t0, std::optional<double> t1) const{
    // First edge case: If no slice arguments are provided, return the full cloud
    if (!t0 && !t1)
        return process ? processed_cloud_ : raw_cloud_;

    // Check if timestamps exist
    if (!raw_cloud_.HasPointAttr("timestamps")) {
        ROS_WARN("Cloud has no timestamp attribute, returning empty cloud.");
        return open3d::t::geometry::PointCloud(); // Return empty cloud
    }

    // Fetch attributes
    std::vector<float> positions = raw_cloud_.GetPointPositions().ToFlatVector<float>();
    std::vector<float> intensities = raw_cloud_.GetPointAttr("intensities").ToFlatVector<float>();
    std::vector<double> timestamps = raw_cloud_.GetPointAttr("timestamps").ToFlatVector<double>();

    // Determine indices
    const int idx0 = t0 ? std::distance(timestamps.begin(), std::lower_bound(timestamps.begin(), timestamps.end(), *t0)) : 0;
    const int idx1 = t1 ? std::distance(timestamps.begin(), std::lower_bound(timestamps.begin(), timestamps.end(), *t1)) : timestamps.size();
    const int n_points = idx1 - idx0;

    // Slice vectors
    positions = std::vector<float>(positions.begin() + idx0 * 3, positions.begin() + idx1 * 3);
    intensities = std::vector<float>(intensities.begin() + idx0, intensities.begin() + idx1);
    timestamps = std::vector<double>(timestamps.begin() + idx0, timestamps.begin() + idx1);

    // Build a map
    if (n_points < 1) {
        ROS_WARN("No points found in the specified time range.");
        return open3d::t::geometry::PointCloud(); // Return empty cloud
    }

    open3d::t::geometry::PointCloud cloud(open3d::core::Tensor(std::move(positions), {n_points, 3}, open3d::core::Dtype::Float32));
    cloud.SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities), {n_points, 1}, open3d::core::Dtype::Float32));
    cloud.SetPointAttr("timestamps", open3d::core::Tensor(std::move(timestamps), {n_points, 1}, open3d::core::Dtype::Float64));

    // Apply processing if requested
    if (process) {
        cloud = cloud_processor_.processCloud(cloud);
    }

    return cloud;
}