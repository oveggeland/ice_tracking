#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"

// Forward declaration
class CloudManager;

using OdometryFactor = gtsam::BetweenFactor<Pose3>;

class LidarOdometry{
public: 
    // Initialize
    LidarOdometry(const ros::NodeHandle& nh, const BatchFixedLagSmoother& smoother);
    void setCloudManager(const CloudManager& cloud_manager){ cloud_manager_ = &cloud_manager; }

    // Estimate odometry from state_idx!
    boost::shared_ptr<gtsam::NonlinearFactor> estimateOdometry(int idx1);
private:
    const CloudManager* cloud_manager_;
    const BatchFixedLagSmoother& smoother_;

    gtsam::noiseModel::Base::shared_ptr noise_model_;
    boost::shared_ptr<gtsam::NonlinearFactor> createOdometryFactor(int idx0, int idx1, Pose3 T_align);
};