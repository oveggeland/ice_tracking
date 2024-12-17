#include "icetrack/cloud_manager.h"

CloudManager::CloudManager(std::shared_ptr<LidarHandle> lidar): point_buffer_(lidar->getSharedBufferPointer()){
    cloud_ = PointCloudBuffer(window_size_ / lidar->getPointInterval());
}

void CloudManager::newPose(double ts, Pose3 pose){
    double dt = ts - ts_prev_; 

    // Query points from last interval
    auto points = point_buffer_->getPointsWithin(ts_prev_, ts);

    for (auto p: points){
        Pose3 wTb = pose_prev_.interpolateRt(pose, (p.ts - ts_prev_) / dt); // Excellent, we have a pose
        
        // Awesome, now add these points efficiently to cloud_ buffer
        PointXYZIT* p_world = cloud_.addPoint();
        p_world->ts = p.ts;
        p_world->intensity = p.intensity;
        
        *reinterpret_cast<Point3*>(p_world) = wTb.transformFrom(*reinterpret_cast<Point3*>(&p));

        if (x0_ == 0.0){
            x0_ = p_world->x;
            y0_ = p_world->y;
        }
        p_world->x -= x0_;
        p_world->y -= y0_;
    }

    // ROS_INFO_STREAM("Cloud manager: Size is " << cloud_.size());

    ts_prev_ = ts;
    pose_prev_ = pose;

    // analyseWindow();
    // saveCloud();
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
    //pcl::io::savePLYFileBinary<pcl::PointXYZI>(fname.str(), *cloud_.toPCLCloud());
};