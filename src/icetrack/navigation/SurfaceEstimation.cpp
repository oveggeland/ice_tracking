#include "icetrack/navigation/SurfaceEstimation.h"

SurfaceEstimation::SurfaceEstimation(){}

SurfaceEstimation::SurfaceEstimation(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    bTl_ = system->bTl();
    point_buffer_ = system->lidar()->getConstBufferPointer();

    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_iterations", ransac_iterations_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_inlier_count", ransac_inlier_count_);

    getParamOrThrow(nh, "/navigation/surface_estimation/frame_interval", frame_interval_);

    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_altitude", sigma_altitude_);
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_attitude", sigma_attitude_);
}


boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimation::getAltitudeFactor(Key key){
    return boost::make_shared<AltitudeFactor>(key, -surface_dist_, noiseModel::Isotropic::Sigma(1, sigma_altitude_));
}

boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimation::getAttitudeFactor(Key key){
    return boost::make_shared<Pose3AttitudeFactor>(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, sigma_attitude_), surface_normal_);
}

double SurfaceEstimation::getSurfaceDistance(){
    return surface_dist_;
}

bool SurfaceEstimation::estimateSurface(double ts){
    // Find bounds in point buffer
    auto start = point_buffer_->iteratorLowerBound(ts - 0.5*frame_interval_);
    auto end = point_buffer_->iteratorLowerBound(ts + 0.5*frame_interval_);

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
    
    // NB: Assuming Lidar is pointing towards the sea, make sure normal vector is pointing downwards.
    if (plane_model[0] < 0)
        plane_model = -plane_model;

    // Transform plane model from Lidar to IMU frame
    surface_normal_ = Unit3(bTl_.rotation().rotate(plane_model.head<3>()));
    surface_dist_ = abs(plane_model[3] - bTl_.translation().dot(surface_normal_.unitVector()));

    return true;
}