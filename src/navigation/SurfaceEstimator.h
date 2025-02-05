#pragma once

#include <open3d/geometry/PointCloud.h>

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh);

    bool estimateSurface(const open3d::geometry::PointCloud& pcd);
    Eigen::Vector4d getPlaneCoeffs() const { return coeffs_; }
private: 
    // Plane model
    Eigen::Vector4d coeffs_;

    // Ransac
    double ransac_frame_size_;
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_inlier_count_;
    int ransac_iterations_;
};