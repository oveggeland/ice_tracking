#include "icetrack/mapping/icemap.h"


IceMap::IceMap(){
    bTl_ = readExtrinsics('L', 'B', "/home/oskar/smooth_sailing/src/smooth_sailing/cfg/calib/ext_right.yaml"); // TODO: Generalize yaml filename
    global_cloud_ = RingBuffer((size_t) 200000*cloud_duration_);
}


void IceMap::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    cloud_buffer_[msg->header.stamp.toSec()] = cloud;
}


void IceMap::addCloud(double t0_cloud, pcl::PointCloud<pcl::PointXYZI> cloud){
    double t1_cloud = t0_cloud + point_interval_*cloud.size();

    gtsam::Pose3 T0, T1;
    if (!queryPose(pose_map_, t0_cloud, T0) || !queryPose(pose_map_, t1_cloud, T1)){
        ROS_WARN("THIS SHOULD NEVER HAPPEN");
        return;
    }

    int point_cnt = 0;
    double int_step = (t1_cloud-t0_cloud) / cloud.size(); // Step size for interpolation coefficient
    for (const auto& point : cloud.points) {
        double d_square = point.x*point.x + point.y*point.y + point.z*point.z;
        if (d_square < min_dist_square_ || d_square > max_dist_square_){ // Rough outlier rejection
            point_cnt ++;
            continue;
        }

        // Get point stamp and pose
        double ts_point = t0_cloud + point_cnt*point_interval_;
        gtsam::Pose3 wTl = T0.interpolateRt(T1, point_cnt*int_step);

        // Transform to world frame
        gtsam::Point3 transformed_point = wTl.transformFrom(gtsam::Point3(point.x, point.y, point.z));

        // Push to ringbuffer
        struct point new_point{
            transformed_point.x(),
            transformed_point.y(),
            transformed_point.z(),
            (uint8_t) point.intensity,
            ts_point
        };
        global_cloud_.addPoint(new_point);

        point_cnt++;
    }

    t_head_ = t1_cloud;
}


void IceMap::checkCloudBuffer(){
    // Iterate through cloud buffer and check timestamp scopes
    for (auto it = cloud_buffer_.begin(); it != cloud_buffer_.end(); ){
        double t0_cloud = it->first;
        double t1_cloud = t0_cloud + point_interval_*it->second.size();

        // Is cloud before first pose? Then erase it
        if (pose_map_.lower_bound(t0_cloud) == pose_map_.begin()){
            // I am old, delete me
            it = cloud_buffer_.erase(it);
        }
        else if (pose_map_.lower_bound(t1_cloud) == pose_map_.end()){
            // I am too young
            it++;
        }
        else{
            addCloud(t0_cloud, it->second);
            it = cloud_buffer_.erase(it);
        }
    }
}


void IceMap::evaluateWindow(){
    global_cloud_.removePointsBefore(t_head_ - cloud_duration_);

    if (t_head_ - t_last_ > cloud_interval_){
        cloud_processor_.processCloud(t_head_, global_cloud_.getPoints());
        t_last_ = t_head_;

        // Convert t_head_ (timestamp) to a string to use as the filename
        std::stringstream fname;
        fname << "/home/oskar/icetrack/output/clouds/" << std::fixed << static_cast<int64_t>(t_head_) << ".ply";

        // Option to save cloud (only XYZI) using the timestamp as filename
        pcl::io::savePLYFileBinary<pcl::PointXYZI>(fname.str(), *global_cloud_.toPCLCloud());
    }
}

void IceMap::updatePoseMap(gtsam::Pose3 body_pose, double t_pose){
    // Normalize translation
    gtsam::Pose3 new_pose = body_pose.compose(bTl_);
    if (x0_.isZero()){
        x0_ = new_pose.translation();
    }

    new_pose = gtsam::Pose3(new_pose.rotation(), new_pose.translation()-x0_);

    // Add to pose map
    pose_map_[t_pose] = new_pose;

    // Maintain pose map
    for (auto it = pose_map_.begin(); it != pose_map_.end();) {
        if (pose_map_.size() <= 5) break;
        pose_map_.erase(it++); // Erase the current element and update the iterator
    }

    
    checkCloudBuffer();
    evaluateWindow();
}