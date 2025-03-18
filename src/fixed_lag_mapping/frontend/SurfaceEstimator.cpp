#include "SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const PointBuffer& point_buffer) : pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Get config from ros params
    getParamOrThrow(nh, "/surface_estimator/window_size", window_size_);
    getParamOrThrow(nh, "/surface_estimator/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/surface_estimator/min_inlier_count", min_inlier_count_);
    
    getParamOrThrow(nh, "/surface_estimator/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/surface_estimator/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/surface_estimator/ransac_iterations", ransac_iterations_);
}

/*
Control block to perform surface estimaton when required by the configuration parameters. 
*/
void SurfaceEstimator::estimateSurface(const int idx){
    double ts; 
    if (!pose_graph_.timeQuery(idx, ts))
        return; // Not available!

    // Get cloud
    auto cloud_ptr = point_buffer_.getCloud(ts - 0.5*window_size_, ts + 0.5*window_size_);
    if (cloud_ptr->points_.size() < min_inlier_count_)
        return;

    // Down sample
    cloud_ptr = cloud_ptr->VoxelDownSample(voxel_size_);
    if (cloud_ptr->points_.size() < min_inlier_count_)
        return;

    // Open3D's built-in RANSAC plane fitting
    auto [plane_coeffs, inliers] = cloud_ptr->SegmentPlane(
        ransac_threshold_,
        ransac_sample_size_,
        ransac_iterations_
    );

    // Check if enough points
    if (inliers.size() < min_inlier_count_)
        return; // Plane fitting failed


    // Ensure a consistent normal direction
    if (plane_coeffs[3] < 0) {
        plane_coeffs = -plane_coeffs;
    }

    // Woho
    pose_graph_.surfaceCallback(idx, plane_coeffs);
}