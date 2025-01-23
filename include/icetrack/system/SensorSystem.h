#pragma once

#include "gtsam/geometry/Pose3.h"
#include "yaml-cpp/yaml.h"

#include "icetrack/system/Imu.h"
#include "icetrack/system/Gnss.h"
#include "icetrack/system/Lidar.h"

#include "icetrack/utils/utils.h"

// Master class for sensor rig
class SensorSystem {
public:
    SensorSystem(ros::NodeHandle nh);

    // Non-const accessors
    Imu& imu() { return imu_; }
    Gnss& gnss() { return gnss_; }
    Lidar& lidar() { return lidar_; }

    // Const accessors
    const Imu& imu() const { return imu_; }
    const Gnss& gnss() const { return gnss_; }
    const Lidar& lidar() const { return lidar_; }

    // Extrinsic accessors
    gtsam::Pose3 bTc() const { return cTb_.inverse(); }
    gtsam::Pose3 bTl() const { return cTb_.inverse().compose(lTc_.inverse()); }
    gtsam::Pose3 cTb() const { return cTb_; }
    gtsam::Pose3 cTl() const { return lTc_.inverse(); }
    gtsam::Pose3 lTb() const { return lTc_.compose(cTb_); }
    gtsam::Pose3 lTc() const { return lTc_; }

private:
    // Private members for sensors
    Imu imu_;
    Gnss gnss_;
    Lidar lidar_;

    // Calibration
    gtsam::Pose3 cTb_;
    gtsam::Pose3 lTc_;

    void readExtrinsics(const std::string& filename);
};