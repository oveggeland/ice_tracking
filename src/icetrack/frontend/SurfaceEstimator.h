#pragma once

#include <open3d/geometry/PointCloud.h>

#include "frontend/PointBuffer.h"

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh, const PointBuffer& point_buffer);

    // Interface
    bool estimateSurface(double ts);
    bool ready(double ts) const {
        return ts - ts_update_ > update_interval_;
    };

    // Accesor
    double getTimeStamp() const { return plane_stamp_; }
    const Eigen::Vector4d& getPlaneCoeffs() const { return plane_coeffs_; }

private: 
    // Reference to necesary resources
    const PointBuffer& point_buffer_;

    // Plane info
    double plane_stamp_;
    Eigen::Vector4d plane_coeffs_;

    double ts_update_ = 0.0;              // Last succesful update

    // General config
    double update_interval_;        // Minimum interval between updates
    double window_size_;            // Size of point cloud window [in seconds] used to fit a plane.
    double voxel_size_;             // Voxel size on downsample (0 means no downsample)
    int min_inlier_count_;          // Minimum amounts of inliers to accept plane

    // Ransac configuration
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_iterations_;
};