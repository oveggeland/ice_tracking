#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>

#include "navigation/navigation.h"

#include "ImuIntegration.h"
#include "GnssCorrection.h"
#include "SurfaceEstimation.h"

#include "factors/NormConstraintFactor.h"
#include "factors/LeveredAltitudeFactor.h"

#include "utils/file_system.h"
#include "utils/ros_params.h"
#include "utils/conversions.h"
#include "utils/CallbackSequencer.h"

using namespace gtsam;

class PoseEstimator{
public:
    PoseEstimator(ros::NodeHandle nh);

private:
    // Subscribers and subscriber callbacks
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Safe callback sequencing
    CallbackSequencer sequencer_;
    
    void imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Private functionality
    void initialize(double ts);
    void initializeState();
    void addPriors();

    void addState(double ts);

    // Factor generating submodules
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;
    SurfaceEstimation surface_estimation_;

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
    
    // General configuration parameters
    double initial_acc_bias_sigma_;
    double initial_gyro_bias_sigma_;
    double lever_norm_threshold_; 
    double lever_norm_sigma_;
    double lever_altitude_sigma_;

    // Output
    ros::Publisher pose_pub_;
    void publishPose();

    std::ofstream f_out_;
    void writeToFile();
};