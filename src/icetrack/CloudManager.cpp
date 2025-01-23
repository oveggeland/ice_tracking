#include "icetrack/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle nh, const SensorSystem& sensors) : bTl_(sensors.bTl()), point_buffer_(sensors.lidar().pointBuffer()) {
    // Get config
    getParamOrThrow(nh, "/cloud/window_size", window_size_);
    getParamOrThrow(nh, "/cloud/window_interval", window_interval_);
    getParamOrThrow(nh, "/cloud/save_cloud", save_cloud_);
    getParamOrThrow(nh, "/cloud/z_lower_bound", z_lower_bound_);
    getParamOrThrow(nh, "/cloud/z_upper_bound", z_upper_bound_);

    cloud_ = StampedRingBuffer<PointDetailed>(window_size_ / sensors.lidar().getPointInterval());

    // Set output paths
    std::string out_path = getParamOrThrow<std::string>(nh, "/outpath");
    
    cloud_path_ = joinPath(out_path, "clouds/");
    makePath(cloud_path_, true);

    stats_path_ = joinPath(out_path, "stats/stats.csv");
    makePath(stats_path_);

    f_stats_ = std::ofstream(stats_path_);
    f_stats_ << "ts,count,z_mean,z_var,z_exp_mean,z_exp_var";
    f_stats_ << std::endl << std::fixed;
}

bool CloudManager::isInit(){
    return init_;
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

void CloudManager::newPose(double t1, gtsam::Pose3 imu_pose){
    // Incoming pose is body, we need lidar pose (because of convention in point_buffer_)
    gtsam::Pose3 pose1 = imu_pose.compose(bTl_);

    if (!isInit()){
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

    for (auto it = start; it != end; ++it){
        double ts = it->ts;
        assert(ts >= t0_);
        assert(ts < t1);

        // Pose interpolation
        double alpha = (ts - t0_)*dt_inv;
        gtsam::Pose3 wTl(
            pose0_.rotation().retract(alpha*dR_log),
            pose0_.translation() + alpha*dt_log
        );

        gtsam::Pose3 wTl_test = pose0_.interpolateRt(pose1, alpha);
        if (!wTl_test.equals(wTl))
            ROS_ERROR("FAILED!");


        // Transform to world frame
        gtsam::Point3 wPwp = wTl.transformFrom(gtsam::Point3(it->x, it->y, it->z));
        if (wPwp.z() < z_lower_bound_ || wPwp.z() > z_upper_bound_)
            continue; // Outlier

        
        // Add to global cloud
        cloud_.addPoint({
            ts,
            wPwp.x(),
            wPwp.y(),
            wPwp.z(),
            it->i, 
        });

        // Adjust moving average and variance
        assert(ts > ts_exp_);

        double beta = exp(-exp_decay_rate_*(ts - ts_exp_));
        assert(beta >= 0 && beta <= 1);

        z_exp_mean_ = beta*z_exp_mean_ + (1 - beta)*wPwp.z();
        double z_dev = z_exp_mean_ - wPwp.z();
        z_exp_var_ = beta*z_exp_var_ + (1 - beta)*(z_dev*z_dev);

        ts_exp_ = ts;
    }

    t0_ = t1;
    pose0_ = pose1;

    //if (t0_ - t0_window_ > window_interval_)
        //analyseWindow();
}

void CloudManager::writeStatistics(){
    f_stats_ << t0_;
    f_stats_ << "," << count_;
    f_stats_ << "," << z_mean_ << "," << z_var_;
    f_stats_ << "," << z_exp_mean_ << "," << z_exp_var_;
    f_stats_ << std::endl;
}

std::shared_ptr<open3d::t::geometry::PointCloud> CloudManager::generateWindowCloud(){
    auto start = cloud_.iteratorLowerBound(t0_ - window_size_);
    auto end = cloud_.iteratorLowerBound(t0_);

    // Create vectors for points and intensity
    std::vector<double> points;   // Flat array for points
    std::vector<float> intensities;

    size_t num_points = start.distance_to(end);
    points.reserve(3*num_points);  // Reserve space for 3 coordinates per point
    intensities.reserve(num_points);

    for (auto it = start; it != end; ++it) {
        points.push_back(it->x);
        points.push_back(it->y);
        points.push_back(it->z);
        intensities.push_back(it->intensity);
    }

    std::unordered_map<std::string, open3d::core::Tensor> tensor_map;
    tensor_map["positions"] = open3d::core::Tensor(points, {(int)points.size()/3, 3}, open3d::core::Dtype::Float64);
    tensor_map["intensities"] = open3d::core::Tensor(intensities, {(int)intensities.size(), 1}, open3d::core::Dtype::Float32);
    
    return std::make_shared<open3d::t::geometry::PointCloud>(tensor_map);
}


void CloudManager::analyseWindow(){
    auto pcd = generateWindowCloud();

    if (save_cloud_)
        saveCloud(pcd);

    writeStatistics();

    t0_window_ = t0_;
}

void CloudManager::saveCloud(std::shared_ptr<open3d::t::geometry::PointCloud> pcd) {
    std::stringstream fname;
    fname << std::fixed << static_cast<int64_t>(t0_) << ".ply";
    std::string fpath = joinPath(cloud_path_, fname.str());

    // Save the point cloud as a .ply file
    if (!open3d::t::io::WritePointCloud(fpath, *pcd))
        ROS_INFO_STREAM("Saved cloud at " << fpath);
    else
        ROS_WARN_STREAM("Failed to save cloud at " << fpath);
}