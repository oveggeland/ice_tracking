#pragma once

#include "ros/ros.h"

#include <proj.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <sensor_msgs/NavSatFix.h>

#include "icetrack/common.h"
#include "icetrack/factors/GNSSFactor.h"

class Gnss{
public:
  Gnss();
  Gnss(ros::NodeHandle nh);

  void init(const sensor_msgs::NavSatFix::ConstPtr& msg);
  bool isInit();

  Point2 getPosition();
  Point2 getVelocity();

  boost::shared_ptr<gtsam::NonlinearFactor> getCorrectionFactor(const sensor_msgs::NavSatFix::ConstPtr& msg, Key key);

private:
  ros::NodeHandle nh_;

  // Control 
  bool init_ = false;

  double ts_ = 0.0;
  Point2 xy_ = Point2::Zero();
  Point2 v_xy_ = Point2::Zero();

  // Noise
  double gnss_sigma_;
  noiseModel::Isotropic::shared_ptr correction_noise_;

  // Projection
  std::string crs_source_;
  std::string crs_target_;
  PJ* projection_;

  // Private functions
  Point2 getMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);
};