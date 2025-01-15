#pragma once

#include "gtsam/geometry/Pose3.h"
#include "yaml-cpp/yaml.h"

#include "icetrack/imu.h"
#include "icetrack/gnss.h"
#include "icetrack/lidar.h"


// Master class
class SensorSystem {
public:
    SensorSystem();
    SensorSystem(ros::NodeHandle nh): nh_(nh){
        // Initialize pointers to different sensors
        imu_ = std::make_shared<Imu>(nh);
        gnss_ = std::make_shared<Gnss>(nh);
        lidar_ = std::make_shared<Lidar>(nh);

        // Initialize extrinsics. TODO: Consider moving to seperate class
        std::string ext_file = getParamOrThrow<std::string>(nh, "/ext_file");
        readExt(ext_file);
    };

    std::shared_ptr<Imu> imu() const { return imu_; }
    std::shared_ptr<Gnss> gnss() const { return gnss_; }
    std::shared_ptr<Lidar> lidar() const { return lidar_; }

    // Functions to get extrinsics
    Pose3 bTc() const {
        return cTb_.inverse();
    }
    Pose3 bTl() const {
        return cTb_.inverse().compose(lTc_.inverse());
    }
    Pose3 cTb() const {
        return cTb_;
    }
    Pose3 cTl() const {
        return lTc_.inverse();
    }
    Pose3 lTb() const {
        return lTc_.compose(cTb_);
    }
    Pose3 lTc() const {
        return lTc_;
    }

private:
    ros::NodeHandle nh_;

    // Private members for sensors
    std::shared_ptr<Imu> imu_;
    std::shared_ptr<Gnss> gnss_;
    std::shared_ptr<Lidar> lidar_;
    
    // Calibration
    gtsam::Pose3 cTb_;
    gtsam::Pose3 lTc_;

    // Read calibration
    void readExt(const std::string& filename){
        YAML::Node config = YAML::LoadFile(filename);

        gtsam::Matrix4 cTb_mat;
        auto Tcb_node = config["T_cam_imu"];

        gtsam::Matrix4 lTc_mat;
        auto Tlc_node = config["T_lidar_cam"];
        
        // Fill the matrix with the values from the YAML node
        for (size_t i = 0; i < 4; i++) {
            for (size_t j = 0; j < 4; j++) {
                cTb_mat(i, j) = Tcb_node[i][j].as<double>();
                lTc_mat(i, j) = Tlc_node[i][j].as<double>();
            }
        }

        cTb_ = gtsam::Pose3(cTb_mat);
        lTc_ = gtsam::Pose3(lTc_mat);
    }
};