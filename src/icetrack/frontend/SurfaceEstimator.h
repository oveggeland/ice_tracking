#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>
#include <chrono>

#include "frontend/PointBuffer.h"
#include "backend/PoseGraph.h"

#include "utils/ros_params.h"

class SurfaceEstimator{
public: 
    // Initialize
    SurfaceEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const PointBuffer& point_buffer);

    // Interface
    void estimateSurface(const int state_idx);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudQuery(const double t0, const double t1) const;
    void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
    bool fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private: 
    // Reference to necesary resources
    PoseGraph& pose_graph_;
    const PointBuffer& point_buffer_;

    // Plane results
    Eigen::Vector4f plane_coeffs_;

    // General config
    double window_size_;            // Size of point cloud window [in seconds] used to fit a plane.
    double voxel_size_;             // Voxel size on downsample (0 means no downsample)
    int min_inlier_count_;          // Minimum amounts of inliers to accept plane

    // Ransac configuration
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_iterations_;

    // PCL stuff
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
};