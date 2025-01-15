#include "icetrack/SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(){}

SurfaceEstimator::SurfaceEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    bTl_ = system->bTl();
    point_buffer_ = system->lidar()->getPointBuffer();

    getParamOrThrow(nh, "/lidar/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/lidar/ransac_prob", ransac_prob_);
    getParamOrThrow(nh, "/lidar/plane_min_inlier_count", min_inlier_count_);

    getParamOrThrow(nh, "/lidar/measurement_interval", measurement_interval_);

    seg_.setModelType(pcl::SACMODEL_PLANE); // Set the model you want to fit
    seg_.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC to estimate the plane
    seg_.setDistanceThreshold(ransac_threshold_);        // Set a distance threshold for points to be considered inliers)
    seg_.setProbability(ransac_prob_);
}

bool SurfaceEstimator::segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    if (cloud->size() < min_inlier_count_){
        return false;
    }

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coeffs);

    if (inliers->indices.size() < min_inlier_count_) {
        return false;
    }
    // Set some parameters based on segmented plane
    z_ = -abs(coeffs->values[3]);
    bZ_ = Unit3(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    if (bTl_.rotation().inverse().rotate(bZ_).point3().x() < 0){ // Assert normal vector has positive x in LiDAR frame (pointing down in world frame)
        bZ_ = Unit3(-bZ_.point3());
    }

    return true;
}

boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimator::getAltitudeFactor(Key key){
    return boost::make_shared<AltitudeFactor>(key, z_, noiseModel::Isotropic::Sigma(1, 1));
}

boost::shared_ptr<gtsam::NonlinearFactor> SurfaceEstimator::getAttitudeFactor(Key key){
    return boost::make_shared<Pose3AttitudeFactor>(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, 0.1), bZ_);
}

bool SurfaceEstimator::isInit(){ 
    return init_;
}

double SurfaceEstimator::getAltitude(){
    return z_;
}

bool SurfaceEstimator::generatePlane(double ts){
    // Timestamp bounds for lidar points to use
    double t0 = ts - 0.5*measurement_interval_;
    double t1 = ts + 0.5*measurement_interval_;

    // Find tail
    // auto cloud_ptr = getPclWithin(t0, t1);
    // if (segmentPlane(cloud_ptr)){
    //     return true;
    // };

    return false;
}

void SurfaceEstimator::init(double ts){
    if (generatePlane(ts))
        init_ = true;
}