#pragma once

#include "ros/ros.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "icetrack/system/SensorSystem.h"

#include "icetrack/navigation/navigation.h"

#include "icetrack/navigation/ImuIntegration.h"
#include "icetrack/navigation/GnssCorrection.h"
#include "icetrack/navigation/SurfaceEstimation.h"

#include "icetrack/navigation/factors/NormConstraintFactor.h"
#include "icetrack/navigation/factors/LeveredAltitudeFactor.h"

#include "icetrack/utils/file_system.h"
#include "icetrack/utils/ros_params.h"

using namespace gtsam;

class PoseEstimator{
public:
    PoseEstimator(ros::NodeHandle nh);

    // Interface
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool isInit() { return init_; }

private:
    // Factor generating submodules
    SurfaceEstimation surface_estimation_;
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;

    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Filter control
    bool init_ = false;
    int state_count_ = 0;

    // State
    double ts_;
    Pose3 pose_;
    Point3 vel_;
    imuBias::ConstantBias bias_;
    Point3 lever_arm_;

    void initializeState();
    void addPriors();

    // Private member functions
    void initialize(double ts);
    void addState(double ts);
    
    
    double initial_acc_bias_sigma_;
    double initial_gyro_bias_sigma_;
    double lever_norm_threshold_; 
    double lever_norm_sigma_;
    double lever_altitude_sigma_;

    // Output
    std::ofstream f_out_;
    void writeToFile();
};