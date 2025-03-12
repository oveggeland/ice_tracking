#include "frontend/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle& nh, PoseGraph& pose_graph) : 
    pose_graph_(pose_graph),
    point_buffer_(nh), 
    frame_buffer_(nh),
    cloud_publisher_(nh),
    surface_estimator_(nh, pose_graph, point_buffer_),
    odometry_estimator_(nh, pose_graph, frame_buffer_){

    // Config
    getParamOrThrow(nh, "/cloud_manager/refine_frames", refine_frames_);
    getParamOrThrow(nh, "/cloud_manager/publish_frames", publish_frames_);
    getParamOrThrow(nh, "/cloud_manager/publish_cloud", publish_cloud_);

    // Setup subscribers
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_sub_ = nh.subscribe(pose_topic, 1, &CloudManager::poseCallback, this);
}


bool CloudManager::generateLidarFrame(const int idx){
    double t0, t1;
    gtsam::Pose3 pose0, pose1;
    
    if (!pose_graph_.timePoseQuery(idx-1, t0, pose0) || !pose_graph_.timePoseQuery(idx, t1, pose1)){
        ROS_WARN_STREAM("Pose not available at idx: " << idx);
        return false;
    }

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Initialize frame with idx and capacity
    LidarFrame& frame = frame_buffer_.emplaceFrame(idx, t1, num_points);//  frame(idx1, num_points);

    // Iterate through and add points
    for (auto it = start; it != end; ++it) {
        frame.addPoint(*it);
    }

    // Undistort
    frame.undistortFirstOrder(t0, t1, pose0, pose1);
    return true;
}


/*
Optional refining (redo undistortion) for all frames with new pose estimates.
*/
void CloudManager::refineFrames(){
    double t0, t1;
    gtsam::Pose3 pose0, pose1;

    for (auto it = frame_buffer_.begin(); it != frame_buffer_.end(); ++it){
        const int frame_id = it->id();
    
        if (pose_graph_.timePoseQuery(frame_id-1, t0, pose0) && 
            pose_graph_.timePoseQuery(frame_id, t1, pose1)){
            // Succesful queries
            it->undistort(t0, t1, pose0, pose1);
        }
    }   
}



/*
Rebuild map based on an updates pose graph and the frame buffer
*/
void CloudManager::rebuildMap(){
    // Clear and reserve 
    const size_t num_points = frame_buffer_.numPoints();
    points_.clear();
    points_.reserve(num_points);
    
    // Iterate through frames 
    gtsam::Pose3 frame_pose; 
    for (auto it = frame_buffer_.cbegin(); it != frame_buffer_.cend(); ++it){
        // First query pose
        if (!pose_graph_.poseQuery(it->id(), frame_pose))
            continue; // Pose not available?

        // Convert to eigen float
        Eigen::Matrix3f R_frame = frame_pose.matrix().topLeftCorner<3, 3>().cast<float>();
        Eigen::Vector3f t_frame = frame_pose.matrix().topRightCorner<3, 1>().cast<float>();

        // Get frame values
        const std::vector<Eigen::Vector3f>& frame_points = it->undistortedPoints();
        const std::vector<float>& frame_intensities = it->intensities();
        const int frame_size = frame_points.size();

        // Add to map
        for (int i = 0; i < frame_size; ++i){
            const Eigen::Vector3f p = R_frame*frame_points[i] + t_frame;
            points_.push_back({
                p.x(),
                p.y(),
                p.z(),
                frame_intensities[i]
            });
        }
    }   
}



/*
On new state events, we do the following steps:
1. Maintain frame buffer (discard old frames)
2. Create a new frame
3. Estimate odometry (frame-to-frame)
4. Rebuild map
5. Publish map
*/
void CloudManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const double ts = msg->header.stamp.toSec();
    const int state_idx = pose_graph_.getCurrentStateIdx();

    // Maintain buffer
    frame_buffer_.maintain();

    // Create new frame
    if (generateLidarFrame(state_idx)){
        odometry_estimator_.estimateOdometry(state_idx);

        // Optional: Publish frame
        if (publish_frames_){
            const LidarFrame& frame = frame_buffer_.back();
            cloud_publisher_.publishFrame(frame.undistortedPoints(), frame.intensities());
        }
    }

    // Try surface estimation (on previous state)
    surface_estimator_.estimateSurface(state_idx - 1);

    // Optional: Refine frames (computationally heavy)
    if (refine_frames_)
        refineFrames();
    
    // Rebuild map
    rebuildMap();

    // Optional: Publish cloud
    if (publish_cloud_)
        cloud_publisher_.publishCloud(points_); 
}

/*
On new lidar measurements, we do the following steps:
1. Add points to buffer
2. Poll surface estimator
*/
void CloudManager::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    point_buffer_.addPoints(msg);
}