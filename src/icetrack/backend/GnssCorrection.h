#pragma once

#include <sensor_msgs/NavSatFix.h>

#include <gtsam/geometry/Point2.h>

#include <proj.h>

#include "utils/ros_params.h"
#include "backend/navigation.h"
#include "backend/factors/GNSSFactor.h"

class GnssCorrection{
public:
    GnssCorrection(ros::NodeHandle nh);

    // Interface
    void newMeasurement(const sensor_msgs::NavSatFix::ConstPtr msg);

    bool isFix() { return fix_; }
    Point2 getPosition() const { return xy_; }

    GNSSFactor getCorrectionFactor(Key key) const { return GNSSFactor(key, xy_, correction_noise_); }

private:
    // State
    double ts_ = 0.0;
    Point2 xy_ = Point2::Zero();

    Point2 xy_offset_ = Point2::Zero();
    bool fix_ = false; 

    bool checkFix(double ts, Point2 xy);

    // Timeout/Suspension logic (GNSS is very innaccurate after timeout)
    double timeout_interval_;
    double suspension_interval_;
    double t0_suspend_ = 0.0;

    // Noise
    noiseModel::Isotropic::shared_ptr correction_noise_;

    // Projection
    PJ* projection_;
    Point2 project(const sensor_msgs::NavSatFix::ConstPtr msg) const;
};