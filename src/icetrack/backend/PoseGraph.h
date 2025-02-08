#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include "navigation.h"
#include "ImuIntegration.h"
#include "GnssCorrection.h"
#include "SurfaceCorrection.h"

#include "factors/NormConstraintFactor.h"
#include "factors/LeveredAltitudeFactor.h"
#include "factors/ConstantVelocityFactor2D.h"
#include "factors/IceOdometryFactor.h"

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
    void odometryCallback(int pose_idx0, int pose_idx1, Eigen::Matrix4d T_align);
    void surfaceCallback(int pose_idx, const Eigen::Vector4d& plane_coeffs);

    // Accessors
    bool isInit() const { return init_; }
    bool exists(int idx) const { return smoother_.getLinearizationPoint().exists(X(idx)); }

    int getCurrentStateIdx() const { return state_idx_; }
    double getCurrentTimeStamp() const { return ts_; }
    const Pose3& getCurrentPose() const { return pose_; }

    double getTimeStamp(int idx) const { return smoother_.timestamps().at(X(idx)); }
    Pose3 getPose(int idx) const { return smoother_.calculateEstimate<Pose3>(X(idx)); }

private:
    // Initialization
    bool init_ = false;

    void initialize();

    // State 
    int state_idx_ = 0;

    void initializeState();
    void addState(double ts);
    void predictState(double ts);
    void updateState(int idx);

    // Graph 
    void addPriors(int idx);
    void updateTimeStamps(int idx, double ts);
    void updateValues(int idx);
    void updateFactors(int idx, double ts);
    void updateSmoother();

    // Optimization
    FixedLagSmoother::KeyTimestampMap stamps_;
    Values values_;
    NonlinearFactorGraph factors_; 

    double lag_;
    BatchFixedLagSmoother smoother_;

    // Factor generator modules
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;
    SurfaceCorrection surface_correction_;

    // State definition
    double ts_ = 0.0;
    Pose3 pose_;
    Point3 vel_ = Point3(0, 0, 0);
    imuBias::ConstantBias bias_;

    bool estimate_lever_arm_ = true;
    Point3 lever_arm_ = Point3(0, 0, 0);

    bool estimate_ice_drift_ = true;
    Point2 ice_drift_ = Point2(0, 0);


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