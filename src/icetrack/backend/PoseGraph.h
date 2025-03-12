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
    bool inRange(double ts) const {
        if (smoother_.timestamps().empty())
            return false;
        return ts > smoother_.timestamps().begin()->second && ts < smoother_.timestamps().rbegin()->second;
    }
    bool exists(int idx) const { return smoother_.getLinearizationPoint().exists(X(idx)); }


    int getCurrentStateIdx() const { return state_.idx; }
    double getCurrentTimeStamp() const { return state_.ts; }
    const Pose3& getCurrentPose() const { return state_.pose; }

    double getTimeStamp(int idx) const { return smoother_.timestamps().at(X(idx)); }
    Pose3 getPose(int idx) const { return smoother_.calculateEstimate<Pose3>(X(idx)); }
    
    Pose3 getPose(double ts) const { // Interpolate to find pose at ts
        Key key0 = 0;
        Key key1 = 0;

        // Iterate over timestamps
        for (auto it = smoother_.timestamps().begin(); it != smoother_.timestamps().end(); ++it){
            if (keyTypeCheck(it->first, 'x')){
                if (it->second > ts){
                    key1 = it->first;
                    break;
                }
                key0 = it->first;
            }
        }

        if (key0 == 0 || key1 == 0){
            ROS_WARN("Pose interpolation key error");
            return Pose3();
        }

        Pose3 pose0 = smoother_.calculateEstimate<Pose3>(key0);
        Pose3 pose1 = smoother_.calculateEstimate<Pose3>(key1);

        double t0 = smoother_.timestamps().at(key0);
        double t1 = smoother_.timestamps().at(key1);

        return pose0.interpolateRt(pose1, (ts-t0)/(t1-t0));
    }


    bool poseQuery(const double ts, Pose3& pose) const {
        if (!isInit() || !inRange(ts))
            return false;
        pose = getPose(ts);
        return true;
    }

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