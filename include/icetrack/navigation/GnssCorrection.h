#pragma once

#include "ros/ros.h"

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "icetrack/utils/utils.h"
#include "icetrack/navigation/navigation.h"
#include "icetrack/navigation/factors/GNSSFactor.h"
#include "icetrack/system/SensorSystem.h"

class GnssCorrection{
public:
    GnssCorrection(ros::NodeHandle nh, const Gnss& gnss);

    bool initialize();    

    bool update();

    double getHead() const;
    Point2 getPosition() const;
    Point2 getVelocity() const;

    GNSSFactor getCorrectionFactor(Key key) const;

private:
    const Gnss& gnss_;

    // State
    double ts_ = 0.0;
    Point2 xy_ = Point2::Zero();
    Point2 v_xy_ = Point2::Zero();

    // Timeout/Suspension logic (GNSS is very innaccurate after timeout)
    double timeout_interval_;
    double suspension_interval_;
    double t0_suspend_ = 0.0;

    // Noise
    noiseModel::Isotropic::shared_ptr correction_noise_;
};