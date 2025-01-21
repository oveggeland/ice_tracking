#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>

#include "icetrack/ShipNavigation.h"
#include "icetrack/navigation/Projection.h"
#include "icetrack/system/SensorSystem.h"
#include "icetrack/utils/utils.h"

using namespace gtsam;


class ShipPoseEstimator{
public: 
    ShipPoseEstimator();
    ShipPoseEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system);

    bool shipUpdate();

    Pose3 getShipPose(const icetrack::ShipNavigation::ConstPtr& msg) const;
    Pose3 predictSystemPose(const icetrack::ShipNavigation::ConstPtr& msg) const;

private:
    std::shared_ptr<Ship> ship_;  
    Pose3 T_align_; // Ship->System alignment matrix
    
    Projection proj_;
};