#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "icetrack/gnss.h"
#include "icetrack/imu.h"
#include "icetrack/lidar.h"

#include "icetrack/altitudeFactor.h"
#include "icetrack/vectorNormFactor.h"
#include "icetrack/angleNormFactor.h"

class IceNav{
public:
    IceNav();
    IceNav(std::shared_ptr<LidarHandle> lidar);

    void imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);

    bool isInit();
    Pose3 getPose();

private:
    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Filter control 
    bool init_ = false;
    int correction_count_ = 0;

    bool request_lidar_correction_ = false;

    // Current state
    double ts_;
    Pose3 pose_;
    Point3 vel_;
    imuBias::ConstantBias bias_;
    Point3 lever_arm_;

    // Sensors
    GnssHandle gnss_;
    ImuHandle imu_;
    std::shared_ptr<LidarHandle> lidar_;

    // Private member functions
    void initialize(double ts);
    void update(double ts);
    
    // Output
    std::ofstream f_nav_;
    void writeToFile();
};