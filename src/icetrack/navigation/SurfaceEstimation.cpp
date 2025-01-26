#include "icetrack/navigation/SurfaceEstimation.h"

SurfaceEstimation::SurfaceEstimation(ros::NodeHandle nh) {
    // Initialize buffer
    double buffer_size = getParamOrThrow<double>(nh, "/navigation/surface_estimation/buffer_size");
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_point_interval", point_interval_);
    point_buffer_ = StampedRingBuffer<PointXYZT>(buffer_size / point_interval_);

    // Read extrinsics
    std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
    bTl_ = bTl(ext_file);

    // Lidar Config
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_max_dist"), 2);

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

void SurfaceEstimation::addLidarFrame(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts_point = msg->header.stamp.toSec() - point_interval_; // Track stamp of points as we iterate
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejections
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
        )
            continue;

        // Range filtering
        double dist_squared = it[0]*it[0] + it[1]*it[1] + it[2]*it[2];
        if (dist_squared < min_dist_squared_ || dist_squared > max_dist_squared_)
            continue;

        // Point accepted
        point_buffer_.addPoint({
            it[0],
            it[1],
            it[2],
            ts_point
        });
    }
    
    if (ts_point > ts_head_){
        ts_head_ = ts_point;
    }
}


AltitudeFactor SurfaceEstimation::getAltitudeFactor(Key pose_key){
    return AltitudeFactor(pose_key, -surface_distance_, noiseModel::Isotropic::Sigma(1, sigma_altitude_));
}

Pose3AttitudeFactor SurfaceEstimation::getAttitudeFactor(Key pose_key){
    return Pose3AttitudeFactor(pose_key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, sigma_attitude_), surface_normal_);
}

bool SurfaceEstimation::estimateSurface(double ts){
    // Find bounds in point buffer
    auto start = point_buffer_.iteratorLowerBound(ts - 0.5*ransac_frame_size_);
    auto end = point_buffer_.iteratorLowerBound(ts + 0.5*ransac_frame_size_);

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
    surface_normal_ = Unit3(bTl_.rotation().rotate(plane_model.head<3>()));
    surface_distance_ = abs(plane_model[3] - bTl_.translation().dot(surface_normal_.unitVector()));

    return true;
}