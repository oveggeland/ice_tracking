#include "icetrack/SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(){}

SurfaceEstimator::SurfaceEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    bTl_ = system->bTl();
    point_buffer_ = system->lidar()->getPointBuffer();

    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_iterations", ransac_iterations_);
    getParamOrThrow(nh, "/navigation/surface_estimation/ransac_inlier_count", ransac_inlier_count_);

    getParamOrThrow(nh, "/navigation/surface_estimation/frame_interval", frame_interval_);

    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_altitude", sigma_altitude_);
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_attitude", sigma_attitude_);
}


boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimator::getAltitudeFactor(Key key){
    return boost::make_shared<AltitudeFactor>(key, -surface_dist_, noiseModel::Isotropic::Sigma(1, sigma_altitude_));
}

boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimator::getAttitudeFactor(Key key){
    return boost::make_shared<Pose3AttitudeFactor>(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, sigma_attitude_), surface_normal_);
}

double SurfaceEstimator::getSurfaceDistance(){
    return surface_dist_;
}

bool SurfaceEstimator::estimateSurface(double ts){
    // Find bounds in point buffer
    auto start = point_buffer_->iteratorLowerBound(ts - 0.5*frame_interval_);
    auto end = point_buffer_->iteratorLowerBound(ts + 0.5*frame_interval_);

    // Check element number
    size_t num_points = start.distance_to(end);
    if (num_points == 0 || num_points < ransac_inlier_count_)
        return false;


    // Extract and transform points from the buffer
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_.reserve(num_points); // Reserve memory for the correct number of points

    for (auto it = start; it != end; ++it) {
        // Add the transformed point to the cloud
        cloud->points_.emplace_back(bTl_.transformFrom(Point3(it->second.x, it->second.y, it->second.z)));
    }

    // Now fit a plane from the cloud
    auto [plane_model, inliers] = cloud->SegmentPlane(ransac_threshold_, ransac_sample_size_, ransac_iterations_);

    // Check if enough inliers were found
    if (inliers.size() < ransac_inlier_count_)
        return false;
    
    // Extract distance and surface normal
    surface_dist_ = abs(plane_model[3]);

    surface_normal_ = Unit3(plane_model.head<3>());
    if (bTl_.rotation().inverse().rotate(surface_normal_).unitVector().x() < 0)
        surface_normal_ = Unit3(-surface_normal_.unitVector());

    return true;
}