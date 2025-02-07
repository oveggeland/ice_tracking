#pragma once

#include <open3d/geometry/PointCloud.h>

#include "frontend/PointBuffer.h"

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh, const PointBuffer& point_buffer);

    // Interface
    bool fitSurface(double ts);

    // Accessors
    const Eigen::Vector4d& getPlaneCoeffs() const { return plane_coeffs_; }
    double getPlaneStamp() const { return ts_plane_; }
    double getPlaneFitStamp() const { return ts_plane_fit_; }

private: 
    const PointBuffer& point_buffer_;

    // Plane model
    Eigen::Vector4d plane_coeffs_;
    bool estimateSurface(double ts);

    // Time stamp management
    double ts_plane_ = 0.0;
    double ts_plane_fit_ = 0.0;

    double plane_interval_ = 60;
    double window_size_ = 0.1;

    // Ransac configuration
    double ransac_frame_size_;
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_inlier_count_;
    int ransac_iterations_;
};