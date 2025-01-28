#include "mapping/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle nh) {
    // Get config
    getParamOrThrow(nh, "/mapping/enabled", enabled_);
    getParamOrThrow(nh, "/mapping/window_size", window_size_);
    getParamOrThrow(nh, "/mapping/save_cloud", save_cloud_);

    getParamOrThrow(nh, "/mapping/min_elevation", min_elevation_);
    getParamOrThrow(nh, "/mapping/max_elevation", max_elevation_);
    getParamOrThrow(nh, "/mapping/min_intensity", min_intensity_);

    getParamOrThrow(nh, "/mapping/grid_size", grid_size_);
    getParamOrThrow(nh, "/mapping/smoothing_radius", smoothing_radius_);
    getParamOrThrow(nh, "/mapping/deformation_radius", deformation_radius_);

    // Setup sequencer and subscribers
    sequencer_ = CallbackSequencer(getParamOrThrow<double>(nh, "/mapping/safe_delay"));

    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");

    lidar_sub_ = nh.subscribe(lidar_topic, 10, &CloudManager::lidarCallback, this);
    pose_sub_ = nh.subscribe(pose_topic, 10, &CloudManager::poseCallback, this);

    // Read extrinsics
    std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
    bTl_ = bTl(ext_file);

    // Allocate cloud storage
    point_buffer_ = StampedRingBuffer<PointXYZIT>(2.0/point_interval_);

    int max_points = window_size_ / point_interval_;
    positions_ = std::vector<float>(3*max_points);
    intensities_ = std::vector<uint8_t>(max_points);

    // Set output paths
    std::string out_path = getParamOrThrow<std::string>(nh, "/outpath");
    
    cloud_path_ = joinPath(out_path, "clouds/");
    makePath(cloud_path_, true);

    stats_path_ = joinPath(out_path, "stats/stats.csv");
    makePath(stats_path_);

    f_stats_ = std::ofstream(stats_path_);
    f_stats_ << "ts,count,z_mean,z_var,i_mean,i_var,freeboard";
    f_stats_ << std::endl << std::fixed;
}

// Subscriber callbacks. Add safe callback version to sequencer for synchronization. 
void CloudManager::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&CloudManager::lidarSafeCallback, this, msg));
}
void CloudManager::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&CloudManager::poseSafeCallback, this, msg));
}


// Add LiDAR points to buffer
void CloudManager::lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (!enabled_)
        return;

    double ts_point = msg->header.stamp.toSec() - point_interval_; // Track stamp of points as we iterate
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejections
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
        )
            continue;

        // Range filtering
        double dist_squared = it[0]*it[0] + it[1]*it[1] + it[2]*it[2];
        if (dist_squared < min_dist_squared_ || dist_squared > max_dist_squared_)
            continue;

        // Point accepted
        point_buffer_.addPoint({
            it[0],
            it[1],
            it[2],
            it[3],
            ts_point
        });
    }
    
    if (ts_point > ts_head_){
        ts_head_ = ts_point;
    }
}


// When a new pose is available, use this to add lidar points from buffer to global cloud
void CloudManager::poseSafeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (!enabled_)
        return;

    double t1 = msg->header.stamp.toSec();
    gtsam::Pose3 imu_pose = poseRosToGtsam(msg->pose);

    // Incoming pose is body, we need lidar pose (because of convention in point_buffer_)
    gtsam::Pose3 pose1 = imu_pose.compose(bTl_);

    if (!init_){
        initialize(t1, pose1);
        return;
    }

    // Normalize XY-coordinates
    pose1 = shiftPose(pose1);

    // Precompute for efficient interpolation in loop 
    gtsam::Vector3 dt_log = pose1.translation() - pose0_.translation();
    gtsam::Vector3 dR_log = gtsam::Rot3::Logmap(pose0_.rotation().between(pose1.rotation()));

    double dt_inv = 1.0 / (t1 - t0_);
    assert(dt_inv > 0); 

    // Find bounds for point buffer iteration
    auto start = point_buffer_.iteratorLowerBound(t0_);
    auto end = point_buffer_.iteratorLowerBound(t1);

    // Iterate over all points
    for (auto it = start; it != end; ++it){
        double ts = it->ts;
        assert(ts >= t0_ && ts < t1);

        // Pose interpolation
        double alpha = (ts - t0_)*dt_inv;
        assert(alpha >= 0 & alpha <= 1);
        gtsam::Pose3 wTl(
            pose0_.rotation().retract(alpha*dR_log),
            pose0_.translation() + alpha*dt_log
        );

        // Transform to world frame
        gtsam::Point3 wPwp = wTl.transformFrom(gtsam::Point3(it->x, it->y, it->z));

        // Initial outlier filtering
        double elevation = -wPwp.z();
        if (elevation < min_elevation_ || elevation > max_elevation_ || it->i < min_intensity_)
            continue; 

        // Save point
        positions_.push_back(wPwp.x());
        positions_.push_back(wPwp.y());
        positions_.push_back(wPwp.z());

        intensities_.push_back(it->i);
    }

    t0_ = t1;
    pose0_ = pose1;

    if (t0_ - t0_window_ > window_size_)
        analyseWindow();
}





gtsam::Pose3 CloudManager::shiftPose(gtsam::Pose3 pose){
    return gtsam::Pose3(pose.rotation(), pose.translation() - gtsam::Point3(x0_, y0_, 0));
}

void CloudManager::initialize(double ts, gtsam::Pose3 pose){
    t0_ = ts;
    t0_window_ = ts;
    x0_ = pose.translation().x();
    y0_ = pose.translation().y();
    pose0_ = shiftPose(pose);
    init_ = true;
}


void CloudManager::analyseWindow(){
    // Generate pointcloud
    auto pcd = open3d::t::geometry::PointCloud();

    // Use move semantics for efficiency
    int num_points = positions_.size() / 3;
    pcd.SetPointPositions(open3d::core::Tensor(std::move(positions_), {num_points, 3}));
    pcd.SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities_), {num_points, 1}));

    // Process cloud
    processCloud(pcd);

    // Save cloud for debugging purposes
    if (save_cloud_)
        saveCloud(pcd);

    // Reset
    t0_window_ = t0_;
    positions_.resize(0);
    intensities_.resize(0);
}

void estimateDeformation(open3d::t::geometry::PointCloud& pcd, double radius) {
    // Convert to legacy PointCloud
    open3d::geometry::PointCloud pcd_legacy = pcd.ToLegacy();
    int num_points = pcd_legacy.points_.size();

    // Build KDTree
    open3d::geometry::KDTreeFlann kdtree(pcd_legacy);

    // Allocate memory to save deformation values
    std::vector<float> deformation(num_points);

    // Loop over each point to calculate deformation
    for (size_t i = 0; i < pcd_legacy.points_.size(); ++i) {
        // Query point
        const Eigen::Vector3d& query_point = pcd_legacy.points_[i];

        // Perform radius search
        std::vector<int> indices;
        std::vector<double> distances;
        int num_neighbors = kdtree.SearchRadius(query_point, radius, indices, distances);

        // Extract z-coordinates of neighbors
        double sum = 0;
        double sum2 = 0;

        for (int idx : indices) {
            double z = pcd_legacy.points_[idx].z();
            sum += z;
            sum2 += z*z;
        }

        double mean = sum / num_neighbors;
        double variance = sum2 / num_neighbors - mean*mean;

        // Store variance as deformation value
        deformation[i] = static_cast<float>(variance);
    }

    pcd.SetPointAttr("deformation", open3d::core::Tensor(std::move(deformation), {num_points, 1}));
}

void smoothElevation(open3d::t::geometry::PointCloud& pcd, double radius) {
    // Convert to legacy 
    auto pcd_legacy = pcd.ToLegacy();
    int num_points = pcd_legacy.points_.size();

    // Build KDTree
    open3d::geometry::KDTreeFlann kdtree(pcd_legacy);

    // Allocate vector for smooth coordinates
    std::vector<float> z_smooth(num_points);

    // Loop over each point to compute the mean Z-coordinate
    for (size_t i = 0; i < num_points; ++i) {
        // Query point
        const Eigen::Vector3d& query_point = pcd_legacy.points_[i];

        // Perform radius search
        std::vector<int> indices;
        std::vector<double> distances;
        int num_neighbors = kdtree.SearchRadius(query_point, radius, indices, distances);

        double z_sum = 0;
        std::vector<double> z_values(num_neighbors);
        for (int idx : indices) {
            z_sum += pcd_legacy.points_[idx].z();
        }

        z_smooth[i] = z_sum / num_neighbors;
    }

    // pcd.SetPointAttr("z_smooth", open3d::core::Tensor(std::move(z_smooth), {num_points, 1}));
    pcd.GetPointPositions().Slice(1, 2, 3) = open3d::core::Tensor(std::move(z_smooth), {num_points, 1});
}



void CloudManager::processCloud(open3d::t::geometry::PointCloud& pcd){
    // Down sample as grid in xy-plane
    gridDownSample(pcd, grid_size_);

    // Smooth out elevation values
    smoothElevation(pcd, smoothing_radius_);

    // Estimate deformation of all points
    estimateDeformation(pcd, deformation_radius_);
}


    // // Estimate moments
    // auto elevation = (-pcd.GetPointPositions().Slice(1, 2, 3)).ToFlatVector<float>(); // Negate z-values
    // auto [z_mean, z_var, z_min, z_max] = estimateMoments(elevation);

    // auto intensities = pcd.GetPointAttr("intensities").ToFlatVector<uint8_t>();
    // auto [i_mean, i_var, i_min, i_max] = estimateMoments(intensities);

    // // Compute histogram for elevation
    // // float p1 = calculatePercentile(elevation, 0.01);
    // // float peak = findHistogramPeak(elevation, 100, z_min, z_max);
    // float freeboard = 0;

    // // Write results
    // f_stats_ << t0_;
    // f_stats_ << "," << count;
    // f_stats_ << "," << z_mean << "," << z_var;
    // f_stats_ << "," << i_mean << "," << i_var;
    // f_stats_ << "," << freeboard;
    // f_stats_ << std::endl;

void CloudManager::saveCloud(const open3d::t::geometry::PointCloud& pcd) {
    std::stringstream fname;
    fname << std::fixed << static_cast<int>(t0_) << ".ply";
    std::string fpath = joinPath(cloud_path_, fname.str());

    // Save the point cloud as a .ply file
    if (open3d::t::io::WritePointCloud(fpath, pcd))
        ROS_INFO_STREAM("Saved cloud at " << fpath);
    else
        ROS_WARN_STREAM("Failed to save cloud at " << fpath);
}