#include "icetrack/CloudManager.h"

CloudManager::CloudManager(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors): 
        point_buffer_(sensors->lidar()->getConstBufferPointer())
{
    bTl_ = sensors->bTl();

    // Get config
    getParamOrThrow(nh, "/cloud/window_size", window_size_);
    getParamOrThrow(nh, "/cloud/window_interval", window_interval_);
    getParamOrThrow(nh, "/cloud/save_cloud", save_cloud_);
    getParamOrThrow(nh, "/cloud/z_lower_bound", z_lower_bound_);
    getParamOrThrow(nh, "/cloud/z_upper_bound", z_upper_bound_);

    cloud_ = StampedRingBuffer<PointDetailed>(window_size_ / sensors->lidar()->getPointInterval());

    // Set output paths
    std::string out_path = getParamOrThrow<std::string>(nh, "/outpath");
    
    cloud_path_ = joinPath(out_path, "clouds/");
    makePath(cloud_path_, true);

    elev_path_ = joinPath(out_path, "elev/");
    makePath(elev_path_, true);

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


void CloudManager::initialize(double t0, gtsam::Pose3 pose0){
    ts_prev_ = t0;
    ts_analysis_ = t0;
    x0_ = pose0.translation().x();
    y0_ = pose0.translation().y();
    pose_prev_ = shiftPose(pose0);
    init_ = true;
}

void CloudManager::newPose(double ts, gtsam::Pose3 body_pose){
    return;
    // Incoming pose is body, we need lidar pose (because of convention in point_buffer_)
    gtsam::Pose3 lidar_pose = body_pose.compose(bTl_);

    if (!isInit()){
        initialize(ts, lidar_pose);
        return;
    }

    // Normalize XY-coordinates
    lidar_pose = shiftPose(lidar_pose);

    // Precompute for efficient interpolation in loop 
    gtsam::Vector3 dt_log = lidar_pose.translation() - pose_prev_.translation();
    gtsam::Vector3 dR_log = gtsam::Rot3::Logmap(pose_prev_.rotation().between(lidar_pose.rotation()));

    double dt_inv = 1.0 / (ts - ts_prev_);
    assert(dt_inv > 0); 

    // Find bounds for point buffer iteration
    auto start = point_buffer_->iteratorLowerBound(ts_prev_);
    auto end = point_buffer_->iteratorLowerBound(ts);

    for (auto it = start; it != end; ++it){
        double p_ts = it->ts;
        assert(p_ts >= ts_prev_);
        assert(p_ts < ts);

        // Pose interpolation
        double alpha = (p_ts - ts_prev_)*dt_inv;
        gtsam::Pose3 wTl(
            pose_prev_.rotation().retract(alpha*dR_log),
            pose_prev_.translation() + alpha*dt_log
        );

        // Pose3 wTl_test = pose_prev_.interpolateRt(lidar_pose, alpha);

        // if (!wTl_test.equals(wTl))
        //     ROS_ERROR("FAILED!");


        // Transform to world frame
        gtsam::Point3 wPwp = wTl.transformFrom(gtsam::Point3(it->x, it->y, it->z));
        if (wPwp.z() < z_lower_bound_ || wPwp.z() > z_upper_bound_)
            continue; // Outlier

        
        // Add to global cloud
        cloud_.addPoint({
            p_ts,
            wPwp.x(),
            wPwp.y(),
            wPwp.z(),
            it->i, 
        });

        // Adjust moving average
        assert(it->ts > ts_exp_);

        double beta = exp(-exp_decay_rate_*(it->ts - ts_exp_));
        assert(beta >= 0 && beta <= 1);

        z_exp_mean_ = beta*z_exp_mean_ + (1 - beta)*wPwp.z();
        double z_dev = z_exp_mean_ - wPwp.z();
        z_exp_var_ = beta*z_exp_var_ + (1 - beta)*(z_dev*z_dev);

        ts_exp_ = it->ts;
    }

    ts_prev_ = ts;
    pose_prev_ = lidar_pose;

    if (ts_prev_ - ts_analysis_ > window_interval_)
        analyseWindow();
}

void CloudManager::writeStatistics(){
    f_stats_ << ts_prev_;
    f_stats_ << "," << count_;
    f_stats_ << "," << z_mean_ << "," << z_var_;
    f_stats_ << "," << z_exp_mean_ << "," << z_exp_var_;
    f_stats_ << std::endl;
}


void CloudManager::analyseWindow(){
    // // Do cloud analysis
    // count_ = 0;
    // double z_sum = 0;
    // double z2_sum = 0;

    // // Get start and end iterators
    // auto start = cloud_.iteratorLowerBound(ts_prev_ - window_size_);
    // auto end = cloud_.iteratorLowerBound(ts_prev_);



    // for (auto it = start; it != end; ++it){
    //     double z = it->z;
    //     z_sum += z;
    //     z2_sum += z*z;

    //     ++count_;
    // }

    // z_mean_ = (z_sum / count_);
    // z_var_ = (z2_sum / count_) - (z_mean_*z_mean_);

    if (save_cloud_)
        saveCloud();

    writeStatistics();

    ts_analysis_ = ts_prev_;
}

void CloudManager::saveCloud() {
    auto start = cloud_.iteratorLowerBound(ts_prev_ - window_size_);
    auto end = cloud_.iteratorLowerBound(ts_prev_);

    // Create vectors for points and intensity
    std::vector<double> points;   // Flat array for points
    std::vector<float> intensities;

    size_t num_points = start.distance_to(end);
    points.reserve(num_points * 3);  // Reserve space for 3 coordinates per point
    intensities.reserve(num_points);

    for (auto it = start; it != end; ++it) {
        // Add points in a flat format (x, y, z)
        points.push_back(it->x);
        points.push_back(it->y);
        points.push_back(it->z);
        intensities.push_back(it->intensity);
    }

    std::unordered_map<std::string, open3d::core::Tensor> tensor_map;
    tensor_map["positions"] = open3d::core::Tensor(points, {(int)points.size()/3, 3}, open3d::core::Dtype::Float64);
    tensor_map["intensities"] = open3d::core::Tensor(intensities, {(int)intensities.size(), 1}, open3d::core::Dtype::Float32);
    open3d::t::geometry::PointCloud cloud(tensor_map);

    // Generate filename
    std::stringstream fname;
    fname << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply";
    std::string fpath = joinPath(cloud_path_, fname.str());
    // Save the point cloud as a .ply file
    if (!open3d::t::io::WritePointCloud(fpath, cloud))
        ROS_INFO_STREAM("Saved cloud at " << fpath);
    else
        ROS_WARN_STREAM("Failed to save cloud at " << fpath);
}