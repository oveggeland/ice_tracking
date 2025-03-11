#include "frontend/LidarFrame.h"


LidarFrame::LidarFrame(const int frame_id, const double ts, const size_t capacity) : id_(frame_id), ts_(ts){
    // Make shared pointers
    distorted_ = std::make_shared<open3d::geometry::PointCloud>();
    undistorted_ = std::make_shared<open3d::geometry::PointCloud>();

    // Reserve capacity
    distorted_->points_.reserve(capacity);
    undistorted_->points_.reserve(capacity);
    intensities_.reserve(capacity);
    dt_.reserve(capacity);
}

void LidarFrame::addPoint(const PointXYZIT& p) {
    distorted_->points_.emplace_back(p.x, p.y, p.z);
    intensities_.push_back(p.i);
    dt_.push_back(ts_ - p.ts);
}

void LidarFrame::undistort(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1){
    // Precompute logmap vectors for interpolation
    const gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    const gtsam::Vector3 dt_log = b1Tb0.translation() / (t1 - t0); // Velcocity
    const gtsam::Vector3 dR_log = gtsam::traits<gtsam::Rot3>::Logmap(b1Tb0.rotation()) / (t1 - t0); // Angular velocity

    // Get point vector references
    const std::vector<Eigen::Vector3d>& distorted_points = distorted_->points_;
    
    std::vector<Eigen::Vector3d>& undistorted_points = undistorted_->points_;
    undistorted_points.clear();

    // Iterate through points, optionally undistorting by interpolation between poses. 
    const int n_points = distorted_points.size();
    for (int i = 0; i < n_points; ++i) {
        // Get point and timedelta
        const Eigen::Vector3d& p_dist = distorted_points[i];
        const double dt = dt_[i];

        // Interpolate alignment
        const gtsam::Pose3 T_align(
            gtsam::traits<gtsam::Rot3>::Expmap(dt*dR_log),
            dt*dt_log
        );
        
        // Push back undistorted point
        undistorted_points.push_back(T_align.transformFrom(p_dist));
    }
}

void LidarFrame::downSample(double voxel_size) {
    ROS_WARN("LidarFrame::downSample not implemented");
}