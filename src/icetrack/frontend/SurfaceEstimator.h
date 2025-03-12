#pragma once

#include <open3d/geometry/PointCloud.h>

#include "frontend/PointBuffer.h"
#include "backend/PoseGraph.h"

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const PointBuffer& point_buffer);

    // Interface
    void estimateSurface(const int state_idx);

private: 
    // Reference to necesary resources
    PoseGraph& pose_graph_;
    const PointBuffer& point_buffer_;

    // General config
    double window_size_;            // Size of point cloud window [in seconds] used to fit a plane.
    double voxel_size_;             // Voxel size on downsample (0 means no downsample)
    int min_inlier_count_;          // Minimum amounts of inliers to accept plane

    // Ransac configuration
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_iterations_;
};