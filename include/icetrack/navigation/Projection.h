#pragma once

#include "ros/ros.h"

#include <proj.h>
#include <gtsam/geometry/Point2.h>

#include "icetrack/common.h"

class Projection{
public:
  Projection();
  Projection(ros::NodeHandle nh);

  gtsam::Point2 project(double latitude, double longitude) const;

private:
  std::string crs_source_;
  std::string crs_target_;
  PJ* projection_;
};