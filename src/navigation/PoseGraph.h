#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include "navigation.h"
#include "ImuIntegration.h"
#include "GnssCorrection.h"

#include "factors/NormConstraintFactor.h"
#include "factors/LeveredAltitudeFactor.h"
#include "factors/ConstantVelocityFactor2D.h"

#include "utils/file_system.h"
#include "utils/ros_params.h"
#include "utils/conversions.h"


class PoseGraph{
public:
    // Constructor
    PoseGraph(ros::NodeHandle& nh);

    // Generic sensor callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    // Measurement callbacks from front end
    void odometryCallback(int pose_idx0, int pose_idx1, Eigen::Matrix4d T_align) {};
    void planeFitCallback(int pose_idx, Eigen::Vector4d plane_coeffs);

    // Accessors
    bool isInit() const { return init_; }
    bool exists(int idx) const { return smoother_.getLinearizationPoint().exists(X(idx)); }

    int getCurrentStateIdx() const { return state_idx_; }
    double getTimeStamp(int idx) const { return smoother_.timestamps().at(X(idx)); }
    Pose3 getPose(int idx) const { return smoother_.calculateEstimate<Pose3>(X(idx)); }

private:
    // When initializing
    void initialize(double ts);

    void initializeState(double ts);
    void addPriors(double ts);

    // When new state are added
    void addState(double ts);

    void predictState(double ts);
    void addVariables(double ts);
    void addFactors(double ts);

    void updateSmoother(double ts);

    // Factor generator modules
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;

    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Filter config
    bool estimate_ice_drift_ = true;
    bool init_ = false;

    // Current state
    double prior_z_ = 0.0;

    int state_idx_ = 0;
    double ts_ = 0.0;
    Pose3 pose_;
    Point3 vel_ = Point3::Zero();
    imuBias::ConstantBias bias_;
    Point3 lever_arm_;
    Point2 ice_drift_;

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