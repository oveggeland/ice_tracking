#pragma once

#include "ros/ros.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "navigation.h"
#include "ImuIntegration.h"
#include "GnssCorrection.h"
#include "SurfaceCorrection.h"
#include "PoseGraphOutput.h"

#include "factors/NormConstraintFactor.h"
#include "factors/LeveredAltitudeFactor.h"

#include "utils/ros_params.h"


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
    const PoseGraphState& getCurrentState() const { return state_; }

    bool exists(int idx) const { return smoother_.getLinearizationPoint().exists(X(idx)); }

    bool poseQuery(const int idx, Pose3& pose) const {
        if (!exists(idx))
            return false;
        pose = smoother_.calculateEstimate<Pose3>(X(idx));
        return true;
    }

    bool timeQuery(const int idx, double& ts) const {
        if (!exists(idx))
            return false;
        
        ts = smoother_.timestamps().at(X(idx));
        return true;
    }

    bool timePoseQuery(const int idx, double& ts, Pose3& pose) const{
        return poseQuery(idx, pose) && timeQuery(idx, ts);
    }

private:
    void readParams(const ros::NodeHandle& nh);

    // Initialization
    bool init_ = false;

    void initialize();
    void initializeState();
    
    void addState(double ts);
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
    int hot_start_delay_;
    IncrementalFixedLagSmoother smoother_;

    // Factor generator modules
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;
    SurfaceCorrection surface_correction_;

    // Defines "current state"
    PoseGraphState state_;

    // General configuration parameters
    double initial_acc_bias_sigma_;
    double initial_gyro_bias_sigma_;
    double lever_norm_threshold_; 
    double lever_norm_sigma_;
    double lever_altitude_sigma_;

    // Output
    PoseGraphOutput pose_graph_output_;
};