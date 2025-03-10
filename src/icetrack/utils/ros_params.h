#pragma once

#include <ros/ros.h>
#include <stdexcept>

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

template <typename T>
void getParamOrDefault(const ros::NodeHandle& nh, const std::string& param_name, T& param, T default_value) {
    if (!nh.getParam(param_name, param))
        param = default_value;
}

template <typename T>
T getParamOrDefault(const ros::NodeHandle& nh, const std::string& param_name, T default_value) {
    T param;
    getParamOrDefault(nh, param_name, param, default_value);
    return param;
}