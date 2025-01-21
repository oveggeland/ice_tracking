#pragma once

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>

#include <gtsam/base/Vector.h>

#include "icetrack/utils/utils.h"
#include "icetrack/navigation/Projection.h"

struct GnssMeasurement{
  double ts=0.0;
  gtsam::Vector2 xy;
  double altitude;
};

class Gnss{
public:
  Gnss();
  Gnss(ros::NodeHandle nh);

  bool newMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);
  const GnssMeasurement& getMeasurement() const;

private:
  GnssMeasurement meas_;

  Projection proj_;

  bool write_to_file_;
  std::ofstream f_out_;
};