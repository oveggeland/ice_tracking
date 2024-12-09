#pragma once

#include "ros/ros.h"

#include <gtsam/geometry/Pose3.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "icetrack/mapping/utils.h"
#include "icetrack/mapping/container.h"
#include "icetrack/mapping/processing.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

class IceMap{
public:
    IceMap();

    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void updatePoseMap(gtsam::Pose3 current_pose, double t_pose);

private:
    void checkCloudBuffer(); // Check if incoming cloud buffer is ready for processing
    void addCloud(double t0_cloud, pcl::PointCloud<pcl::PointXYZI> cloud);
    void evaluateWindow();

    std::map<double, gtsam::Pose3> pose_map_;
    std::map<double, pcl::PointCloud<pcl::PointXYZI>> cloud_buffer_; // Incoming clouds

    RingBuffer global_cloud_; // Container for the global pointcloud

    gtsam::Pose3 bTl_; // LiDAR to IMU
    gtsam::Point3 x0_ = gtsam::Point3(0, 0, 0); // For relative coordinates

    double min_x_dist_ = 5.0;           // Outlier rejection
    double point_interval_ = 5.0e-6;    // Interval between points in dual return mode

    double t_head_ = 0.0;               // Keep track of "head" of pointcloud 
    double t_last_ = 0.0;                   // Starting point

    double cloud_duration_ = 20.0;
    double cloud_interval_ = 10.0;

    CloudProcessor cloud_processor_;
};