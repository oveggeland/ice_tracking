#pragma once

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include "LidarBuffer.h"

#include "navigation.h"
#include "ImuIntegration.h"
#include "GnssCorrection.h"
#include "SurfaceEstimation.h"
#include "LidarOdometry.h"

#include "factors/NormConstraintFactor.h"
#include "factors/LeveredAltitudeFactor.h"
#include "factors/ConstantVelocityFactor2D.h"

#include "utils/file_system.h"
#include "utils/ros_params.h"
#include "utils/conversions.h"


// Forward declarations
class CloudManager;

class PoseGraphManager{
public:
    // Initialize
    PoseGraphManager(ros::NodeHandle& nh);
    void setCloudManager(CloudManager& cloud_manager);

    // Callbacks
    int imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    int gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    // Interface to cloud manager
    std::tuple<double, Pose3> getStampedPose(int idx);
    Point2 getIceDrift(int idx);

private:
    // Interface to cloud manager
    CloudManager* cloud_manager_;

    // When initializing
    void initialize(double ts);

    void initializeStates(double ts);
    void addPriors();


    // When new state are added
    int addState(double ts);

    void addFactors(double ts);
    void predictStates(double ts);
    void updateSmoother(double ts);

    // Factor generator modules
    ImuIntegration imu_integration_;
    GnssCorrection gnss_correction_;
    SurfaceEstimation surface_estimation_;
    LidarOdometry lidar_odometry_;

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

    Point2 ice_drift_;
    bool estimate_ice_drift_ = true;

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