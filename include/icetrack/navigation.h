#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "icetrack/ShipNavigation.h"

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "icetrack/SensorSystem.h"
#include "icetrack/gnss.h"
#include "icetrack/imu.h"
#include "icetrack/SurfaceEstimator.h"

#include "icetrack/factors/NormConstraintFactor.h"
#include "icetrack/factors/AngularConstraintFactor.h"
#include "icetrack/factors/LeveredAltitudeFactor.h"

#include "icetrack/file_system.h"

class IceNav{
public:
    IceNav();
    IceNav(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors);

    void imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void shipNavigationMeasurement(const icetrack::ShipNavigation::ConstPtr& msg);

    bool isInit();
    Pose3 getPose();

private:
    ros::NodeHandle nh_;
    
    // Surface plane fitting
    SurfaceEstimator surface_estimator_;

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
    std::shared_ptr<SensorSystem> sensors_;
    Gnss gnss_;
    Imu imu_;
    std::shared_ptr<Lidar> lidar_;

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

    double gnss_innovation_norm_limit_;

    // Ship
    bool use_ship_nav_;

    // Output
    std::ofstream f_nav_;
    void writeToFile();
};