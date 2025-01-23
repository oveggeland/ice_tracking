#pragma once

#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <iostream>

#ifndef DEG2RAD
#define DEG2RAD(degrees) ((degrees) * M_PI / 180.0)
#endif

#ifndef RAD2DEG
#define RAD2DEG(radians) ((radians) * 180.0 / M_PI)
#endif

template <typename T>
void getParamOrThrow(const ros::NodeHandle& nh, const std::string& param_name, T& param) {
    if (!nh.getParam(param_name, param)) {
        throw std::runtime_error("Failed to get parameter '" + param_name + "'. Ensure it is set in your parameter server.");
    }
}

template <typename T>
T getParamOrThrow(const ros::NodeHandle& nh, const std::string& param_name) {
    T param;
    getParamOrThrow<T>(nh, param_name, param);
    return param;
}

