#pragma once

#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class IceMap{
public:
    IceMap();

    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

private:
    int test_;
};