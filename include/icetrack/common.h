#pragma once

#include <ros/ros.h>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>

using namespace gtsam;

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (bLbs - lever arm from body to ship buyonacy center)

template <typename T>
void getParamOrThrow(const ros::NodeHandle& nh, const std::string& param_name, T& param_value) {
    if (!nh.getParam(param_name, param_value)) {
        throw std::runtime_error("Failed to get parameter '" + param_name + "'. Ensure it is set in your parameter server.");
    }
}