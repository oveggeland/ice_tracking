#include "icetrack/cloud_manager.h"

CloudManager::CloudManager(ros::NodeHandle nh, std::shared_ptr<LidarHandle> lidar): 
        nh_(nh), point_buffer_(lidar->getSharedBufferPointer())
{
    // Get config
    getParamOrThrow(nh_, "/cloud/window_size", window_size_);
    getParamOrThrow(nh_, "/cloud/window_interval", window_interval_);
    getParamOrThrow(nh_, "/cloud/save_cloud", save_cloud_);
    getParamOrThrow(nh_, "/cloud/z_lower_bound", z_lower_bound_);
    getParamOrThrow(nh_, "/cloud/z_upper_bound", z_upper_bound_);

    cloud_ = PointCloudBuffer(window_size_ / lidar->getPointInterval());

    // Set output paths
    std::string out_path = getParamOrThrow<std::string>(nh_, "/outpath");
    
    cloud_path_ = joinPath(out_path, "clouds/");
    makePath(cloud_path_, true);

    stats_path_ = joinPath(out_path, "stats/stats.csv");
    makePath(stats_path_);

    f_stats_ = std::ofstream(stats_path_);
    f_stats_ << "ts,count,z_mean,z_var,i_mean,i_var,z_exp_mean,z_exp_var";
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


void CloudManager::newPose(double ts, Pose3 pose){
    if (!isInit()){
        initialize(ts, pose);
        return;
    }

    // Normalize XY-coordinates
    pose = shiftPose(pose);

    // Assert time stamp correctness
    double dt = ts - ts_prev_;
    assert(dt > 0);

    auto start = point_buffer_->iteratorLowerBound(ts_prev_);
    auto end = point_buffer_->iteratorLowerBound(ts);

    for (auto it = start; it != end; ++it){
        // Check time stamp
        assert(it->ts >= ts_prev_);
        assert(it->ts < ts);

        // Interpolate pose
        Pose3 wTb = pose_prev_.interpolateRt(pose, (it->ts - ts_prev_) / dt); // TODO: Make more efficient (Logmap() in interpolation can be done before the loop)
        Point3 r_world = wTb.transformFrom(Point3(it->x, it->y, it->z));

        if (r_world.z() < z_lower_bound_ || r_world.z() > z_upper_bound_)
            continue; // Outlier

        cloud_.addPoint({
            r_world.x(),
            r_world.y(),
            r_world.z(),
            it->intensity, 
            it->ts
        });

        // Adjust moving average
        assert(it->ts > ts_exp_);

        double alpha = exp(-exp_decay_rate_*(it->ts - ts_exp_));
        assert(alpha >= 0 && alpha <= 1);

        z_exp_mean_ = alpha*z_exp_mean_ + (1 - alpha)*r_world.z();
        double z_dev = z_exp_mean_ - r_world.z();
        z_exp_var_ = alpha*z_exp_var_ + (1 - alpha)*(z_dev*z_dev);

        ts_exp_ = it->ts;
    }

    ts_prev_ = ts;
    pose_prev_ = pose;

    if (ts_prev_ - ts_analysis_ > window_interval_)
        analyseWindow();

    writeStatistics();
}


void CloudManager::calculateMoments(){
    double z_sum = 0;
    float i_sum = 0;

    auto start = cloud_.iteratorLowerBound(ts_prev_ - window_size_);
    auto end = cloud_.iteratorLowerBound(ts_prev_);

    count_ = 0;
    for (auto it = start; it != end; ++it){
        z_sum += it->z;
        i_sum += it->intensity;
        ++ count_;
    }

    z_mean_ = z_sum / count_;
    i_mean_ = i_sum / count_;

    z_sum = 0;
    i_sum = 0;
    for (auto it = start; it != end; ++it){
        z_sum += pow((it->z - z_mean_), 2);
        i_sum += pow((it->intensity - i_mean_), 2);
    }

    z_var_ = z_sum / count_;
    i_var_ = i_sum / count_;
}

void CloudManager::writeStatistics(){
    f_stats_ << ts_prev_;
    f_stats_ << "," << count_;
    f_stats_ << "," << z_mean_ << "," << z_var_;
    f_stats_ << "," << i_mean_ << "," << i_var_;
    f_stats_ << "," << z_exp_mean_ << "," << z_exp_var_;
    f_stats_ << std::endl;
}


void CloudManager::analyseWindow(){
    // Do cloud analysis
    calculateMoments();

    if (save_cloud_)
        saveCloud();

    ts_analysis_ = ts_prev_;
}


void CloudManager::saveCloud(){
    std::stringstream fname;
    fname << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply";

    pcl::io::savePLYFileBinary<pcl::PointXYZI>(joinPath(cloud_path_, fname.str()), *cloud_.getPclWithin(ts_prev_ - window_size_, ts_prev_));
};