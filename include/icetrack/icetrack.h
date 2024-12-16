#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "icetrack/gnss.h"
#include "icetrack/imu.h"
#include "icetrack/lidar.h"
#include "icetrack/diagnostics.h"

#include "icetrack/altitudeFactor.h"
#include "icetrack/vectorNormFactor.h"
#include "icetrack/angleNormFactor.h"

class IceTrack{
public:
    IceTrack(){};
    IceTrack(ros::NodeHandle nh, double lag);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Time control
    double t_head_ = 0.0;       // Last processed message
    double t_safe_ = 0.0;       // Earliest time allowed for processing a message
    double safe_delay_ = 0.5;   // Delay between latest incoming message timestamp and t_safe_

    // Saving incoming measurements in buffer and process in chronological time after safe_delay has expired
    std::map<double, std::function<void()>> callback_buffer_;
    void addCallback(double ts, std::function<void()> cb);
    void checkCallbackBuffer();

    // Filter control 
    double lag_;
    bool init_ = false;
    int correction_count_ = 0;

    bool request_lidar_correction_ = false;

    // Current state
    double ts_;
    Pose3 pose_;
    Point3 vel_;
    imuBias::ConstantBias bias_;
    Point3 lever_arm_;

    // Sensors
    GnssHandle gnss_;
    ImuHandle imu_;
    LidarHandle lidar_;

    Diagnostics diag_;
    
    // Private member functions
    void initialize(double ts);
    void update(double ts);

    void imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclMeasurement(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // Output
    std::ofstream f_nav_;
    void writeToFile();
};