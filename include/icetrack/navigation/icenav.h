#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/navigation/AttitudeFactor.h"

#include "icetrack/navigation/gnss.h"
#include "icetrack/navigation/imu.h"
#include "icetrack/navigation/altitudeFactor.h"


class IceNav{
public:
    IceNav();

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    NonlinearFactorGraph graph_; 
    Values values_;

    bool init_ = false;
    int correction_count_ = 0;
    std::vector<double> correction_stamps_;

    GnssHandle gnss_handle_;
    ImuHandle imu_handle_;

    void finish();
};