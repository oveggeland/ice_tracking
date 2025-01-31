#include "SurfaceEstimation.h"

SurfaceEstimation::SurfaceEstimation(const ros::NodeHandle& nh, const LidarBuffer& lidar_buffer) : lidar_buffer_(lidar_buffer){
    // Read extrinsics
    std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
    bTl_ = bTl(ext_file);

    // Ransac config
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_frame_size", ransac_frame_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_iterations", ransac_iterations_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_inlier_count", ransac_inlier_count_);

    // Factor config
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_altitude", sigma_altitude_);
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_attitude", sigma_attitude_);
}

bool SurfaceEstimation::estimateSurface(double ts){
    // Find bounds in point buffer
    auto start = lidar_buffer_.lowerBoundPointIterator(ts - 0.5*ransac_frame_size_);
    auto end = lidar_buffer_.lowerBoundPointIterator(ts + 0.5*ransac_frame_size_);

    // Check number of elements in window
    int num_points = start.distance_to(end);
    if (num_points < ransac_inlier_count_){
        return false;
    }

    // Generate point cloud
    open3d::geometry::PointCloud cloud;
    cloud.points_.reserve(num_points);

    for (auto it = start; it != end; ++it){
        cloud.points_.emplace_back(it->x, it->y, it->z);
    }

    // Downsample (in space)
    auto cloud_downsampled = cloud.VoxelDownSample(1.0);
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

    // Transform plane model from Lidar to IMU frame
    normal_ = Unit3(bTl_.rotation().rotate(plane_model.head<3>()));
    distance_ = plane_model[3] - bTl_.translation().dot(normal_.unitVector());

    return true;
}



AltitudeFactor SurfaceEstimation::getAltitudeFactor(Key pose_key) const{
    return AltitudeFactor(pose_key, -getSurfaceDistance(), noiseModel::Isotropic::Sigma(1, sigma_altitude_));
}

Pose3AttitudeFactor SurfaceEstimation::getAttitudeFactor(Key pose_key) const{
    return Pose3AttitudeFactor(pose_key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, sigma_attitude_), getSurfaceNormal());
}