#include "icetrack/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle nh, const SensorSystem& sensors) : bTl_(sensors.bTl()), point_buffer_(sensors.lidar().pointBuffer()) {
    // Get config
    getParamOrThrow(nh, "/cloud/window_size", window_size_);
    getParamOrThrow(nh, "/cloud/save_cloud", save_cloud_);
    getParamOrThrow(nh, "/cloud/z_lower_bound", z_lower_bound_);
    getParamOrThrow(nh, "/cloud/z_upper_bound", z_upper_bound_);

    int max_points = window_size_ / sensors.lidar().getPointInterval();
    positions_ = std::vector<double>(3*max_points);
    intensities_ = std::vector<float>(max_points);

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

CloudManager::~CloudManager(){
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

        // Outlier rejection
        if (wPwp.z() < z_lower_bound_ || wPwp.z() > z_upper_bound_)
            continue;

        // Save values
        positions_.push_back(wPwp.x());
        positions_.push_back(wPwp.y());
        positions_.push_back(wPwp.z());
        intensities_.push_back(it->i);

        // // Adjust moving average and variance
        // assert(ts > ts_exp_);

        // double beta = exp(-exp_decay_rate_*(ts - ts_exp_));
        // assert(beta >= 0 && beta <= 1);

        // z_exp_mean_ = beta*z_exp_mean_ + (1 - beta)*wPwp.z();
        // double z_dev = z_exp_mean_ - wPwp.z();
        // z_exp_var_ = beta*z_exp_var_ + (1 - beta)*(z_dev*z_dev);

        // ts_exp_ = ts;
    }

    t0_ = t1;
    pose0_ = pose1;

    if (t0_ - t0_window_ > window_size_)
        analyseWindow();
}

void CloudManager::writeStatistics(){
    f_stats_ << t0_;
    f_stats_ << "," << count_;
    f_stats_ << "," << z_mean_ << "," << z_var_;
    f_stats_ << "," << z_exp_mean_ << "," << z_exp_var_;
    f_stats_ << std::endl;
}

void CloudManager::analyseWindow(){
    auto pcd = open3d::t::geometry::PointCloud();

    // Use move semantics for efficienty
    int num_points = intensities_.size();
    pcd.SetPointPositions(open3d::core::Tensor(std::move(positions_), {num_points, 3}));
    pcd.SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities_), {num_points, 1}));

    if (save_cloud_)
        saveCloud(pcd);

    // writeStatistics();

    // Reset window
    positions_.resize(0);
    intensities_.resize(0);
    t0_window_ = t0_;
}

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