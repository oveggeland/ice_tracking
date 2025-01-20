#pragma once

#include "ros/ros.h"

#include "InertialPoseEstimator.h"
#include "ShipPoseEstimator.h"

using namespace gtsam;

class PoseEstimator{
public:
    PoseEstimator();
    PoseEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors);

    bool imuUpdate();
    bool gnssUpdate();
    bool lidarUpdate();

    bool shipUpdate();

    Pose3 getPose() const;

private:
    // Surface plane fitting
    InertialPoseEstimator inertial_pose_estimator_;
    ShipPoseEstimator ship_pose_estimator_;

    // Ship
    bool use_ship_estimate_;
};