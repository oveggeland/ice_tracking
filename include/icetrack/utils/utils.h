#pragma once

#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <filesystem>
#include <iostream>
#include <fstream>


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

namespace fs = std::filesystem;

// Creates a path if it does not exist. Optionally clears previous content.
inline void makePath(const std::string& path, bool clean = false) {
    fs::path p(path);

    // If the path includes a filename, extract the parent path
    if (p.has_filename()) {
        p = p.parent_path();
    }
    
    // Create the directory (and intermediate directories) if needed
    fs::create_directories(p);
    std::cout << "Directory prepared: " << p << '\n';

    if (clean) {
        // Clear contents of the directory
        for (const auto& entry : fs::directory_iterator(p)) {
            fs::remove_all(entry.path());
            std::cout << "Removed: " << entry.path() << '\n';
        }
    }
}

// Joins two strings to form a valid path.
inline std::string joinPath(const std::string& base, const std::string& sub) {
    return (fs::path(base) / fs::path(sub)).string();
}