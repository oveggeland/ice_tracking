#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/navigation/AttitudeFactor.h"

#include <gtsam/nonlinear/BatchFixedLagSmoother.h>

#include "icetrack/navigation/gnss.h"
#include "icetrack/navigation/imu.h"
#include "icetrack/navigation/altitudeFactor.h"

class IceNav{
public:
    IceNav(){};
    IceNav(ros::NodeHandle nh, double lag);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    Pose3 getLastPose(double& t_pose); // Last pose variable

private:
    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Control
    double lag_;
    bool init_ = false;
    int correction_count_ = 0;

    // Current state
    double ts_;
    Pose3 pose_;
    Point3 vel_;
    imuBias::ConstantBias bias_;

    // Sensors
    GnssHandle gnss_;
    ImuHandle imu_;

    // Private member functions
    void initialize(double ts);
    void update(double ts);
    
    // Output
    std::ofstream f_out_;
    void writeToFile();
};