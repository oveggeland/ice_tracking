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

#include "icetrack/factors/NormConstraintFactor.h"
#include "icetrack/factors/AngularConstraintFactor.h"
#include "icetrack/factors/LeveredAltitudeFactor.h"

#include "icetrack/file_system.h"

class IceNav{
public:
    IceNav();
    IceNav(ros::NodeHandle nh, std::shared_ptr<LidarHandle> lidar);

    void imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);

    bool isInit();
    Pose3 getPose();

private:
    ros::NodeHandle nh_;
    
    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;
    double fixed_lag_;

    // Filter control 
    bool init_ = false;
    int correction_count_ = 0;

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
    
    double initial_bias_sigma_;
    double initial_velocity_sigma_;
    double initial_position_sigma_;
    
    double lever_norm_threshold_; 
    double lever_angle_threshold_; // In deg
    double lever_norm_sigma_;
    double lever_angle_sigma_;
    double lever_altitude_sigma_;


    // Output
    std::ofstream f_nav_;
    void writeToFile();
};