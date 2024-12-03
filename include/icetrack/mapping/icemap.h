#pragma once

#include "ros/ros.h"

#include <gtsam/geometry/Pose3.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "icetrack/mapping/utils.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class IceMap{
public:
    IceMap();

    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void updatePoseMap(gtsam::Pose3 current_pose, double t_pose);

private:
    void transformClouds();

    std::map<double, gtsam::Pose3> pose_map_;
    std::map<double, pcl::PointCloud<pcl::PointXYZI>> cloud_buffer_; // Incoming clouds

    pcl::PointCloud<pcl::PointXYZI> global_cloud_; // Global cloud

    gtsam::Pose3 bTl_; // LiDAR to IMU

    double point_interval_ = 5.0e-6; // Interval between points in dual return mode
};