#pragma once

#include "ros/ros.h"
#include "gtsam/geometry/Pose3.h"

#include "icetrack/common.h"
#include "icetrack/ShipNavigation.h"

class Ship{
    public:
        Ship(){};
        Ship(ros::NodeHandle nh){
            // Initialize ship to body
            std::vector<double> alignment_rotation = getParamOrThrow<std::vector<double>>(nh, "/system/ship/alignment_rotation");
            std::vector<double> alignment_translation = getParamOrThrow<std::vector<double>>(nh, "/system/ship/alignment_translation");
            T_align_ = gtsam::Pose3(
                gtsam::Rot3::RzRyRx(DEG2RAD(alignment_rotation[0]), DEG2RAD(alignment_rotation[1]), DEG2RAD(alignment_rotation[2])),
                gtsam::Point3(alignment_translation[0], alignment_translation[1], alignment_translation[2])
            );
        }

    private:
        gtsam::Pose3 T_align_;
};