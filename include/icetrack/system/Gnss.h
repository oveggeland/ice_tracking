#pragma once

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>

#include <gtsam/geometry/Point2.h>

#include "icetrack/utils/file.h"
#include "icetrack/navigation/Projection.h"

class Gnss{
public:
  Gnss();
  Gnss(ros::NodeHandle nh);

  void newMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);

  const double& getTimeStamp() const;
  const gtsam::Point2& getPosition() const;

private:
  double ts_;
  gtsam::Point2 xy_;

  Projection proj_;

  bool write_to_file_;
  std::ofstream f_out_;
};