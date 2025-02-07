/*
This class represents the interface needed to build cloud from incoming lidar messages and pose estimates. 
*/
#pragma once

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/conversions.h"
#include "utils/StampedRingBuffer.h"


struct PointXYZITDouble{
    double x,y,z;
    float i;
    double ts;
};


class CloudGenerator{
public:
    CloudGenerator(const ros::NodeHandle& nh);

    // Add LiDAR points to buffer
    void addLidarFrame(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // New pose, now we can transform lidar points!
    void newPose(const geometry_msgs::PoseStamped::ConstPtr& pose);

    // Generate a cloud with points from time interval (t0, t1)
    open3d::t::geometry::PointCloud generatePointCloud(double t0, double t1) const;

private:
    // Lidar point buffer
    double ts_head_ = 0.0;                                                                          // Timestamp of last point added to buffer
    double point_interval_;                                                                         // Interval between each point
    double point_buffer_size_;                                                                      // Buffer size in seconds
    StampedRingBuffer<PointXYZIT> point_buffer_;                                                    // Ringbuffer for storage of raw lidar points
    
    bool isValidPoint(const sensor_msgs::PointCloud2ConstIterator<float>& it, double ts_point) const;     // Validity check for incoming points
    bool isWithinRange(float x, float y, float z) const;                                                  // Validity check for incoming points

    // Global (world frame) cloud buffer
    gtsam::Pose3 bTl_;
    StampedRingBuffer<PointXYZITDouble> cloud_buffer_;

    // Keep track of previous pose for interpolation
    double t_pose_prev_ = 0.0;
    gtsam::Pose3 pose_prev_;
    void initialize(double t0, gtsam::Pose3 pose0);

    // Keep track of offset
    bool shift_cloud_;
    double x_shift_;
    double y_shift_;
    gtsam::Pose3 shiftPose(const gtsam::Pose3& pose) const;

    // Filtering conditions
    double min_dist_squared_;
    double max_dist_squared_;
    double min_elevation_;
    double max_elevation_;
    float min_intensity_;
};