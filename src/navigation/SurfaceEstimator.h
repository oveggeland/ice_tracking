#pragma once

#include <open3d/geometry/PointCloud.h>

#include "PointBuffer.h"

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh, const PointBuffer& point_buffer);

    // Interface
    bool estimateSurface(int state_idx, double ts);

    // Accessors
    int getPlaneIndex() const { return plane_idx_; }
    const Eigen::Vector4d& getPlaneCoeffs() const { return plane_coeffs_; }
private: 
    const PointBuffer& point_buffer_;

    // Plane model
    double plane_ts_;
    int plane_idx_ = -1;
    Eigen::Vector4d plane_coeffs_;

    double plane_interval_ = 0.1;

    // Ransac configuration
    double ransac_frame_size_;
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_inlier_count_;
    int ransac_iterations_;
};