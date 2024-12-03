#include "icetrack/mapping/icemap.h"


IceMap::IceMap(){
    bTl_ = readExtrinsics('L', 'B', "/home/oskar/smooth_sailing/src/smooth_sailing/cfg/calib/ext_right.yaml"); // TODO: Generalize yaml filename
}


void IceMap::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    cloud_buffer_[msg->header.stamp.toSec()] = cloud;
}


void IceMap::transformClouds(){
    // Iterate through cloud buffer and check scope
    for (auto it = cloud_buffer_.begin(); it != cloud_buffer_.end(); ){
        double t0_frame = it->first;
        double t1_frame = t0_frame + point_interval_*it->second.size();

        // Is cloud before first pose? Then erase it
        if (pose_map_.lower_bound(t0_frame) == pose_map_.begin()){
            // I am old, delete me
            it = cloud_buffer_.erase(it);
        }
        else if (pose_map_.lower_bound(t1_frame) == pose_map_.end()){
            // I am too young
            it++;
        }
        else{
            // Here is within bounds!
            gtsam::Pose3 T0, T1;
            if (!queryPose(pose_map_, t0_frame, T0) || !queryPose(pose_map_, t1_frame, T1)){
                it++; // Something weird happened?
                ROS_WARN("THIS SHOULD NEVER HAPPEN");
                continue;
            }

            int point_cnt = 0;
            for (const auto& point : it->second.points) {
                if (point.x < 5 || point.intensity < 0){ // Rough outlier rejection
                    point_cnt ++;
                    continue;
                }

                // Get point stamp and pose
                double ts_point = t0_frame + point_cnt*point_interval_;
                gtsam::Pose3 T = T0.interpolateRt(T1, ts_point);

                // Transform to world frame
                gtsam::Point3 transformed_point = T.transformFrom(gtsam::Point3(point.x, point.y, point.z));

                // TODO: Push point to global cloud
                pcl::PointXYZI pcl_point;
                pcl_point.x = transformed_point.x();
                pcl_point.y = transformed_point.y();
                pcl_point.z = transformed_point.z();
                pcl_point.intensity = point.intensity;

                // Push point to global cloud
                global_cloud_.points.push_back(pcl_point);

                point_cnt++;
            }

            it = cloud_buffer_.erase(it);
        }
    }

    // Maybe do some cloud handling

    ROS_WARN_STREAM("Global cloud size is: " << global_cloud_.size());
}

void IceMap::updatePoseMap(gtsam::Pose3 pose, double t_pose){
    ROS_INFO_STREAM(std::fixed << t_pose << ": New pose");

    pose_map_[t_pose] = pose;

    for (auto it = pose_map_.begin(); it != pose_map_.end();) {
        if (pose_map_.size() <= 5) break;
        pose_map_.erase(it++); // Erase the current element and update the iterator
    }

    transformClouds();
}