#include "icetrack/system/SensorSystem.h"

SensorSystem::SensorSystem(ros::NodeHandle nh)
    : imu_(nh), gnss_(nh), lidar_(nh) {
    std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
    readExtrinsics(ext_file);
}

void SensorSystem::readExtrinsics(const std::string& filename){
    // Get YAML nodes
    YAML::Node config = YAML::LoadFile(filename);
    auto cTb_node = config["T_cam_imu"];
    auto lTc_node = config["T_lidar_cam"];
    
    // Fill the matrix with the values from the YAML node
    gtsam::Matrix4 cTb_mat, lTc_mat;
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            cTb_mat(i, j) = cTb_node[i][j].as<double>();
            lTc_mat(i, j) = lTc_node[i][j].as<double>();
        }
    }

    // Convert to Pose3 objects
    cTb_ = gtsam::Pose3(cTb_mat);
    lTc_ = gtsam::Pose3(lTc_mat);
}