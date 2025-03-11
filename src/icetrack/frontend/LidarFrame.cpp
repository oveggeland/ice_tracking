#include "frontend/LidarFrame.h"


LidarFrame::LidarFrame(const int frame_id, const double ts, const size_t capacity) : id_(frame_id), ts_(ts){
    // Reserve capacity
    distorted_points_.reserve(capacity);
    undistorted_points_.reserve(capacity);
    intensities_.reserve(capacity);
    dt_.reserve(capacity);
}

void LidarFrame::addPoint(const PointXYZIT& p) {
    distorted_points_.emplace_back(p.x, p.y, p.z);
    intensities_.push_back(p.i);
    dt_.push_back(ts_ - p.ts);
}


void LidarFrame::undistortFirstOrder(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1){
    // Precompute logmap vectors for interpolation
    const gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    
    const Eigen::Vector3f dp_dt = b1Tb0.translation().cast<float>() / (t1 - t0); // Velcocity
    const Eigen::Matrix3f dR_dt =  gtsam::SO3::Hat(gtsam::Rot3::Logmap(b1Tb0.rotation())).cast<float>() / (t1 - t0); // Angular velocity 
    const Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();

    // Clear and reserve undisorted vector
    const int n_points = distorted_points_.size();
    undistorted_points_.clear();
    undistorted_points_.reserve(n_points);

    // Iterate through points, optionally undistorting by interpolation between poses. 
    for (int i = 0; i < n_points; ++i) {
        // Get alignment
        const double dt = dt_[i];
        const Eigen::Matrix3f R_align = I3 + dt*dR_dt; // First order approximation
        const Eigen::Vector3f t_align = dt*dp_dt;

        undistorted_points_.emplace_back(R_align*distorted_points_[i] + t_align);
    }
}

void LidarFrame::undistort(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1){
    // Precompute logmap vectors for interpolation
    const gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    const gtsam::Vector3 dt_log = b1Tb0.translation() / (t1 - t0); // Velcocity
    const gtsam::Vector3 dR_log = gtsam::traits<gtsam::Rot3>::Logmap(b1Tb0.rotation()) / (t1 - t0); // Angular velocity

    // Get point vector references
    const int n_points = distorted_points_.size();
    undistorted_points_.clear();
    undistorted_points_.reserve(n_points);

    // Iterate through points, optionally undistorting by interpolation between poses. 
    for (int i = 0; i < n_points; ++i) {
        // Get point and timedelta
        const Eigen::Vector3d& p_dist = distorted_points_[i].cast<double>();
        const double dt = dt_[i];

        // Interpolate alignment
        const gtsam::Pose3 T_align(
            gtsam::traits<gtsam::Rot3>::Expmap(dt*dR_log),
            dt*dt_log
        );
        
        // Push back undistorted point
        undistorted_points_.emplace_back(T_align.transformFrom(p_dist).cast<float>());
    }
}