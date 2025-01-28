#pragma once

#include "gtsam/geometry/Pose3.h"
#include "yaml-cpp/yaml.h"

inline gtsam::Pose3 readExtrinsics(const std::string& filename, const std::string& label){
    // Get YAML nodes
    YAML::Node config = YAML::LoadFile(filename);
    auto T_node = config[label];
    
    // Fill the matrix with the values from the YAML node
    gtsam::Matrix4 T_mat;
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            T_mat(i, j) = T_node[i][j].as<double>();
        }
    }

    // Convert to Pose3 objects
    return gtsam::Pose3(T_mat);
}

// Camera <-> IMU
inline gtsam::Pose3 cTb(const std::string& filename){
    return readExtrinsics(filename, "T_cam_imu");
}
inline gtsam::Pose3 bTc(const std::string& filename){ return cTb(filename).inverse(); }

// Camera <-> Lidar
inline gtsam::Pose3 lTc(const std::string& filename){
    return readExtrinsics(filename, "T_lidar_cam");
}
inline gtsam::Pose3 cTl(const std::string& filename){ return lTc(filename).inverse(); }

// IMU <-> Lidar
inline gtsam::Pose3 bTl(const std::string& filename){ return bTc(filename).compose(cTl(filename)); }
inline gtsam::Pose3 lTb(const std::string& filename){ return bTl(filename).inverse(); }