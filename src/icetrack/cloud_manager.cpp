#include "icetrack/cloud_manager.h"

CloudManager::CloudManager(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors): 
        nh_(nh), point_buffer_(sensors->lidar()->getPointBuffer())
{
    bTl_ = sensors->bTl();

    // Get config
    getParamOrThrow(nh_, "/cloud/window_size", window_size_);
    getParamOrThrow(nh_, "/cloud/window_interval", window_interval_);
    getParamOrThrow(nh_, "/cloud/save_cloud", save_cloud_);
    getParamOrThrow(nh_, "/cloud/z_lower_bound", z_lower_bound_);
    getParamOrThrow(nh_, "/cloud/z_upper_bound", z_upper_bound_);

    cloud_ = StampedRingBuffer<PointDetailed>(window_size_ / sensors->lidar()->getPointInterval());

    // Set output paths
    std::string out_path = getParamOrThrow<std::string>(nh_, "/outpath");
    
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


Pose3 CloudManager::shiftPose(Pose3 pose){
    return Pose3(pose.rotation(), pose.translation() - Point3(x0_, y0_, 0));
}


void CloudManager::initialize(double t0, Pose3 pose0){
    ts_prev_ = t0;
    ts_analysis_ = t0;
    x0_ = pose0.translation().x();
    y0_ = pose0.translation().y();
    pose_prev_ = shiftPose(pose0);
    init_ = true;
}

void CloudManager::newPose(double ts, Pose3 body_pose){
    // Incoming pose is body, we need lidar pose (because of convention in point_buffer_)
    Pose3 lidar_pose = body_pose.compose(bTl_);

    if (!isInit()){
        initialize(ts, lidar_pose);
        return;
    }

    // Normalize XY-coordinates
    lidar_pose = shiftPose(lidar_pose);

    // Precompute for efficient interpolation in loop 
    Vector3 dt_log = traits<Point3>::Logmap(traits<Point3>::Between(pose_prev_.translation(), lidar_pose.translation()));
    Vector3 dR_log = traits<Rot3>::Logmap(traits<Rot3>::Between(pose_prev_.rotation(), lidar_pose.rotation()));

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
        Pose3 wTl = Pose3(
            pose_prev_.rotation().Retract(alpha*dR_log),
            pose_prev_.translation() + alpha*dt_log
        );

        // Transform to world frame
        Point3 wPwp = wTl.transformFrom(Point3(it->x, it->y, it->z));
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
        // assert(it->ts > ts_exp_);

        // double alpha = exp(-exp_decay_rate_*(it->ts - ts_exp_));
        // assert(alpha >= 0 && alpha <= 1);

        // z_exp_mean_ = alpha*z_exp_mean_ + (1 - alpha)*r_world.z();
        // double z_dev = z_exp_mean_ - r_world.z();
        // z_exp_var_ = alpha*z_exp_var_ + (1 - alpha)*(z_dev*z_dev);

        // ts_exp_ = it->ts;
    }

    ts_prev_ = ts;
    pose_prev_ = lidar_pose;

    // if (ts_prev_ - ts_analysis_ > window_interval_)
    //     analyseWindow();
}

void CloudManager::writeStatistics(){
    f_stats_ << ts_prev_;
    f_stats_ << "," << count_;
    f_stats_ << "," << z_mean_ << "," << z_var_;
    f_stats_ << "," << z_exp_mean_ << "," << z_exp_var_;
    f_stats_ << std::endl;
}


void CloudManager::analyseWindow(){
    // Do cloud analysis
    count_ = 0;
    double z_sum = 0;
    double z2_sum = 0;

    // Get start and end iterators
    auto start = cloud_.iteratorLowerBound(ts_prev_ - window_size_);
    auto end = cloud_.iteratorLowerBound(ts_prev_);



    for (auto it = start; it != end; ++it){
        double z = it->z;
        z_sum += z;
        z2_sum += z*z;

        ++count_;
    }

    z_mean_ = (z_sum / count_);
    z_var_ = (z2_sum / count_) - (z_mean_*z_mean_);

    if (save_cloud_)
        saveCloud();

    writeStatistics();

    ts_analysis_ = ts_prev_;
}


void CloudManager::saveCloud(){
    std::stringstream fname;
    fname << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply";
};