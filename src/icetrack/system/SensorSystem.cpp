#include "icetrack/system/SensorSystem.h"

SensorSystem::SensorSystem(){};

SensorSystem::SensorSystem(ros::NodeHandle nh){
    // Initialize pointers to different sensors
    imu_ = std::make_shared<Imu>(nh);
    gnss_ = std::make_shared<Gnss>(nh);
    lidar_ = std::make_shared<Lidar>(nh);
    ship_ = std::make_shared<Ship>(nh);

    // Initialize extrinsics
    std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
    readExtrinsics(ext_file);
};


std::shared_ptr<Imu> SensorSystem::imu() const { 
    return imu_; 
}
std::shared_ptr<Gnss> SensorSystem::gnss() const { 
    return gnss_; 
}
std::shared_ptr<Lidar> SensorSystem::lidar() const { 
    return lidar_; 
}
std::shared_ptr<Ship> SensorSystem::ship() const { 
    return ship_; 
}


gtsam::Pose3 SensorSystem::bTc() const {
    return cTb_.inverse();
}
gtsam::Pose3 SensorSystem::bTl() const {
    return cTb_.inverse().compose(lTc_.inverse());
}
gtsam::Pose3 SensorSystem::cTb() const {
    return cTb_;
}
gtsam::Pose3 SensorSystem::cTl() const {
    return lTc_.inverse();
}
gtsam::Pose3 SensorSystem::lTb() const {
    return lTc_.compose(cTb_);
}
gtsam::Pose3 SensorSystem::lTc() const {
    return lTc_;
}


void SensorSystem::readExtrinsics(const std::string& filename){
    // Get YAML nodes
    YAML::Node config = YAML::LoadFile(filename);
    auto Tcb_node = config["T_cam_imu"];
    auto Tlc_node = config["T_lidar_cam"];
    
    // Fill the matrix with the values from the YAML node
    gtsam::Matrix4 cTb_mat, lTc_mat;
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            cTb_mat(i, j) = Tcb_node[i][j].as<double>();
            lTc_mat(i, j) = Tlc_node[i][j].as<double>();
        }
    }

    // Convert to Pose3 objects
    cTb_ = gtsam::Pose3(cTb_mat);
    lTc_ = gtsam::Pose3(lTc_mat);
}