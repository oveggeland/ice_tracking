#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>

#include <gtsam/navigation/AttitudeFactor.h>
#include "backend/factors/AltitudeFactor.h"

#include "backend/navigation.h"
#include "utils/ros_params.h"

class SurfaceCorrection{
public: 
    // Initialize
    SurfaceCorrection(const ros::NodeHandle& nh);

    void setPlaneCoeffs(const Eigen::Vector4d& coeffs) { plane_coeffs_ = coeffs; plane_count_++; }   

    double getSurfaceDistance() const { return abs(plane_coeffs_[3]); }
    Unit3 getSurfaceNormal() const { return Unit3(plane_coeffs_.head<3>()); }

    int planeCount() const { return plane_count_; }

    AltitudeFactor getAltitudeFactor(Key pose_key) const;
    Pose3AttitudeFactor getAttitudeFactor(Key pose_key) const;

private:
    // Plane parameters
    int plane_count_ = 0;
    Eigen::Vector4d plane_coeffs_;

    // Factor noise
    double sigma_altitude_;
    double sigma_attitude_;
};