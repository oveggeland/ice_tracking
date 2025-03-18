#include "SurfaceCorrection.h"

SurfaceCorrection::SurfaceCorrection(const ros::NodeHandle& nh){
    // Factor config
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_altitude", sigma_altitude_);
    getParamOrThrow(nh, "/navigation/surface_estimation/sigma_attitude", sigma_attitude_);
}

AltitudeFactor SurfaceCorrection::getAltitudeFactor(Key pose_key) const{
    return AltitudeFactor(pose_key, -getSurfaceDistance(), noiseModel::Isotropic::Sigma(1, sigma_altitude_));
}

Pose3AttitudeFactor SurfaceCorrection::getAttitudeFactor(Key pose_key) const{
    return Pose3AttitudeFactor(pose_key, Unit3(0, 0, -1), noiseModel::Isotropic::Sigma(2, sigma_attitude_), getSurfaceNormal());
}