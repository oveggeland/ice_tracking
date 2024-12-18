#include "icetrack/cloud_manager.h"

CloudManager::CloudManager(std::shared_ptr<LidarHandle> lidar): point_buffer_(lidar->getSharedBufferPointer()){
    cloud_ = PointCloudBuffer(window_size_ / lidar->getPointInterval());
}


bool CloudManager::isInit(){
    return init_;
}


Pose3 CloudManager::shiftPose(Pose3 pose){
    return Pose3(pose.rotation(), pose.translation() - Point3(x0_, y0_, 0));
}


void CloudManager::initialize(double t0, Pose3 pose0){
    ts_prev_ = t0;
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
        Pose3 wTb = pose_prev_.interpolateRt(pose, (it->ts - ts_prev_) / dt);
        Point3 r_world = wTb.transformFrom(Point3(it->x, it->y, it->z));

        cloud_.addPoint({
            r_world.x(),
            r_world.y(),
            r_world.z(),
            it->intensity, 
            it->ts
        });
    }

    ts_prev_ = ts;
    pose_prev_ = pose;

    //saveCloud();
}


// void CloudManager::calculateMoments(){
//     z_min_ = cloud_[0].z;
//     z_max_ = z_min_;
    
//     double sum = 0;
//     for (auto p: cloud_){
//         sum += p.z;

//         if (p.z < z_min_)
//             z_min_ = p.z;
//         else if (p.z > z_max_)
//             z_max_ = p.z;
//     }
//     z_mean_ = sum / cloud_.size();

//     sum = 0;
//     for (auto p: cloud_){
//         double err = p.z - z_mean_;
//         sum += err*err;
//     }
//     z_var_ = sum / cloud_.size();
// }

void CloudManager::analyseWindow(){
    // Get window in PCL format
    auto window = cloud_.getPclWithin(ts_prev_ - window_size_, ts_prev_);
    
    // Save as binary
    std::stringstream fname;
    fname << "/home/oskar/icetrack/output/clouds/" << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply";
    ROS_INFO_STREAM("/home/oskar/icetrack/output/clouds/" << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply");
    pcl::io::savePLYFileBinary<pcl::PointXYZI>(fname.str(), *window);

    // Do cloud analysis
    //calculateMoments();
    // writeStatistics();
}


void CloudManager::saveCloud(){
    std::stringstream fname;
    fname << "/home/oskar/icetrack/output/clouds/" << std::fixed << static_cast<int64_t>(ts_prev_) << ".ply";
    pcl::io::savePLYFileBinary<pcl::PointXYZI>(fname.str(), *cloud_.getPclWithin(ts_prev_ - window_size_, ts_prev_));
};