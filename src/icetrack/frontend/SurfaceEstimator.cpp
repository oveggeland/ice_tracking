#include "SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const PointBuffer& point_buffer) : 
    pose_graph_(pose_graph), point_buffer_(point_buffer) {

    // Get config from ros params
    getParamOrThrow(nh, "/surface_estimator/update_interval", update_interval_);
    getParamOrThrow(nh, "/surface_estimator/window_size", window_size_);
    getParamOrThrow(nh, "/surface_estimator/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/surface_estimator/min_inlier_count", min_inlier_count_);
    
    getParamOrThrow(nh, "/surface_estimator/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/surface_estimator/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/surface_estimator/ransac_iterations", ransac_iterations_);
}


void SurfaceEstimator::pollUpdates(){
    // Get timestamp and state idx from pose graph
    double ts = pose_graph_.getCurrentTimeStamp();
    int state_idx = pose_graph_.getCurrentStateIdx();

    // Already tried this?
    if (plane_stamp_ == ts)
        return;

    // If not initialized, go ahead. Else we adopt a minimum interval
    if (!pose_graph_.isInit() || (ts - ts_update_ > update_interval_))
        estimateSurface(state_idx, ts);
}


void SurfaceEstimator::estimateSurface(int state_idx, double ts){
    // Establish time interval for plane fitting
    double t0 = ts - 0.5*window_size_;
    double t1 = ts + 0.5*window_size_;

    // Check if buffer has the range we need
    if ((point_buffer_.tail() > t0) || (point_buffer_.head() < t1))
        return;

    // Set default plane result 
    plane_stamp_ = ts;
    plane_idx_ = state_idx;
    plane_coeffs_.setZero();

    // Fetch cloud and increment counter (if the estimation does not work not, it never will)
    auto cloud = point_buffer_.getPointCloud(t0, t1);
    if (cloud->points_.size() < min_inlier_count_)
        return;

    // Downsample
    auto cloud_downsampled = cloud->VoxelDownSample(voxel_size_);
    if (cloud_downsampled->points_.size() < min_inlier_count_)
        return;

    // Fit plane with RANSAC
    auto [plane_model, inliers] = cloud_downsampled->SegmentPlane(ransac_threshold_, ransac_sample_size_, ransac_iterations_);

    // Check if enough inliers were found
    if (inliers.size() < min_inlier_count_)
        return;
    
    // NB: Plane convention check. Make sure we choose (A, B, C, D) such that (A, B, C) is pointing upwards from ice field (i.e. D > 0)
    plane_coeffs_ = plane_model[3] < 0 ? -plane_model : plane_model;
    
    // Update pose graph
    pose_graph_.surfaceCallback(plane_idx_, plane_coeffs_);
    ts_update_ = ts;
}