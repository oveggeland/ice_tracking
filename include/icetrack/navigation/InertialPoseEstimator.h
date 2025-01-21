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

#include "icetrack/utils/utils.h"

using namespace gtsam;

class InertialPoseEstimator{
public:
    // Constructors
    InertialPoseEstimator();
    InertialPoseEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors);

    // Interface
    bool imuUpdate();
    bool gnssUpdate();
    bool lidarUpdate();

    Pose3 getPose() const;
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
    int correction_count_ = 0;

    // Current state
    double ts_;
    Pose3 pose_;
    Point3 vel_;
    imuBias::ConstantBias bias_;
    Point3 lever_arm_;

    // Private member functions
    void initialize(double ts);
    void newCorrection(double ts);
    
    
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