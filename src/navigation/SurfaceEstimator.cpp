#include "SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(const ros::NodeHandle& nh, const PointBuffer& point_buffer) : point_buffer_(point_buffer) {
    // Ransac config
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_frame_size", ransac_frame_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_iterations", ransac_iterations_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_inlier_count", ransac_inlier_count_);
}

bool SurfaceEstimator::estimateSurface(int state_idx, double ts){
    // Establish time interval for plane fitting
    double t0 = ts - 0.5*plane_interval_;
    double t1 = ts + 0.5*plane_interval_;

    // Check if buffer has the range we need
    if ((point_buffer_.tail() > t0) || (point_buffer_.head() < t1))
        return false;

    // Fetch cloud and increment counter (if the estimation does not work not, it never will)
    auto cloud = point_buffer_.getPointCloud(t0, t1);
    plane_idx_ = state_idx;

    // Downsample (in space)
    auto cloud_downsampled = cloud->VoxelDownSample(1.0);
    if (cloud_downsampled->points_.size() < ransac_inlier_count_){
        return false;
    }

    // Fit plane with RANSAC
    auto [plane_model, inliers] = cloud_downsampled->SegmentPlane(ransac_threshold_, ransac_sample_size_, ransac_iterations_);

    // Check if enough inliers were found
    if (inliers.size() < ransac_inlier_count_){
        return false;
    }
    
    // NB: Plane convention check. Make sure we choose (A, B, C, D) such that (A, B, C) is pointing upwards from ice field (i.e. D > 0)
    if (plane_model[3] < 0)
        plane_model = -plane_model;
    plane_coeffs_ = plane_model;

    return true;
}