#pragma once

#include "gtsam/geometry/Pose3.h"
#include "yaml-cpp/yaml.h"

#include "icetrack/system/Imu.h"
#include "icetrack/system/Gnss.h"
#include "icetrack/system/Lidar.h"
#include "icetrack/system/Ship.h"

#include "icetrack/utils/utils.h"

// Master class
class SensorSystem {
public:
    SensorSystem();
    SensorSystem(ros::NodeHandle nh);

    std::shared_ptr<Imu> imu() const; 
    std::shared_ptr<Gnss> gnss() const; 
    std::shared_ptr<Lidar> lidar() const; 
    std::shared_ptr<Ship> ship() const;

    // Functions to get extrinsics
    gtsam::Pose3 bTc() const;
    gtsam::Pose3 bTl() const;
    gtsam::Pose3 cTb() const;
    gtsam::Pose3 cTl() const;
    gtsam::Pose3 lTb() const;
    gtsam::Pose3 lTc() const;

private:
    // Private members for sensors
    std::shared_ptr<Imu> imu_;
    std::shared_ptr<Gnss> gnss_;
    std::shared_ptr<Lidar> lidar_;
    std::shared_ptr<Ship> ship_;
    
    // Calibration
    gtsam::Pose3 cTb_;
    gtsam::Pose3 lTc_;

    // Read calibration
    void readExtrinsics(const std::string& filename);
};