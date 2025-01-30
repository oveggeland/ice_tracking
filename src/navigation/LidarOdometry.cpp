#include "navigation/LidarOdometry.h"

LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, const LidarBuffer& lidar_buffer) : point_buffer_(lidar_buffer.constBufferReference()){
    // Read extrinsics
    bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file"));
}

void LidarOdometry::addFrame(int state_idx, double t0, double t1, Pose3 pose0, Pose3 pose1){
    // Retrieve pointcloud 
    auto pcd = createPointCloud(t0, t1, pose0, pose1);
    
}

open3d::geometry::PointCloud LidarOdometry::createPointCloud(double t0, double t1, const Pose3& pose0, const Pose3& pose1) const{
    assert(t1 > t0);
    double dt_inv = 1 / (t1 - t0);

    // Convert to lidar pose
    Pose3 pose0_lidar = pose0.compose(bTl_);
    Pose3 pose1_lidar = pose1.compose(bTl_);

    // Precompute logmap differences between the two poses (for efficient interpolation)
    Vector3 dt_log = pose0_lidar.translation() - pose0_lidar.translation();
    Vector3 dR_log = Rot3::Logmap(pose1_lidar.rotation().between(pose1_lidar.rotation()));

    // Find bounds for point buffer iteration
    auto start = point_buffer_.iteratorLowerBound(t0);
    auto end = point_buffer_.iteratorLowerBound(t1);

    // Initialize cloud
    open3d::geometry::PointCloud cloud;
    cloud.points_.reserve(start.distance_to(end));

    // Transform to pose0_lidar and add to cloud
    for (auto it = start; it != end; ++it){
        double ts_point = it->ts;
        assert(ts_point >= t0 && ts_point < t1);

        // Pose interpolation
        double alpha = (ts_point - t0)*dt_inv;
        assert(alpha >= 0 & alpha <= 1);
        
        Pose3 T_align_lidar(
            Rot3::Expmap(alpha*dR_log),
            alpha*dt_log
        );
        Pose3 T_align_imu = bTl_.compose(T_align_lidar);

        // Transform to initial IMU pose
        gtsam::Point3 point = T_align_imu.transformFrom(Point3(it->x, it->y, it->z));

        // Add to cloud
        cloud.points_.emplace_back(point.x(), point.y(), point.z());
    }

    return cloud;
}