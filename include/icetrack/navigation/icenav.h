#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/navigation/AttitudeFactor.h"

#include <gtsam/nonlinear/BatchFixedLagSmoother.h>

#include "icetrack/navigation/gnss.h"
#include "icetrack/navigation/imu.h"
#include "icetrack/navigation/altitudeFactor.h"


class IceNav{
public:
    IceNav();

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool isFinished(){return finished_;}

private:
    // Optimization
    NonlinearFactorGraph graph_; 
    Values values_;

    double lag_;
    BatchFixedLagSmoother smoother_;
    FixedLagSmoother::KeyTimestampMap stamps_;

    // Control
    bool init_ = false;
    bool finished_ = false;
    int correction_count_ = 0;
    std::vector<double> correction_stamps_;

    Pose3 prev_pose_;
    Point3 prev_vel_;
    imuBias::ConstantBias prev_bias_;

    std::ofstream f_out_;

    // Sensors
    GnssHandle gnss_handle_;
    ImuHandle imu_handle_;

    // Private member functions
    void initialize(double ts, Point2 initial_xy);
    void update(double ts);

    void finish();
};