#include "icetrack/cloud_manager.h"

CloudManager::CloudManager(std::shared_ptr<LidarHandle> lidar): point_buffer_(lidar->getSharedBufferPointer()){
    cloud_ = PointCloudBuffer(window_size_ / lidar->getPointInterval());
}

void CloudManager::newPose(double ts, Pose3 pose){
    if (ts_prev_ == 0.0){
        ts_prev_ = ts;
        x0_ = pose.translation().x();
        y0_ = pose.translation().y();
        pose_prev_ = Pose3(pose.rotation(), pose.translation() - Point3(x0_, y0_, 0));
        return;
    }

    pose = Pose3(pose.rotation(), pose.translation() - Point3(x0_, y0_, 0));



    double dt = ts - ts_prev_;
    assert(dt > 0);

    for (auto it = point_buffer_->iteratorLowerBound(ts_prev_); it != point_buffer_->iteratorLowerBound(ts); ++it){
        double ts_point = it->ts;
    
        assert(ts_point >= ts_prev_);
        assert(ts_point < ts);

        Pose3 wTb = pose_prev_.interpolateRt(pose, (ts_point - ts_prev_) / dt); // Excellent, we have a pose
        Point3 vec_world = wTb.transformFrom(Point3(it->x, it->y, it->z));

        PointXYZIT p {
            vec_world.x(),
            vec_world.y(),
            vec_world.z(),
            it->intensity,
            ts_point
        };
        cloud_.addPoint(p);    
    }

    ts_prev_ = ts;
    pose_prev_ = pose;

    // analyseWindow();
    saveCloud();
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