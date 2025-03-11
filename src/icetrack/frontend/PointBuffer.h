#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "utils/pointcloud.h"

#include "utils/calibration.h"
#include "utils/ros_params.h"
#include "utils/StampedRingBuffer.h"

#include "point_types.h"

using PointBufferType = StampedRingBuffer<PointXYZIT>;
using PointBufferIterator = StampedRingBufferIterator<PointXYZIT>;

class PointBuffer{
public:
    // Constructor
    PointBuffer(const ros::NodeHandle& nh);

    // Interface
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Accessors
    double head() const { return ts_head_; }
    double tail() const { return begin()->ts; }

    PointCloudPtr getPointCloud(double t0, double t1) const;

    PointBufferIterator begin() const { return buffer_.begin(); }
    PointBufferIterator end() const { return buffer_.end(); }
    PointBufferIterator lowerBound(double ts) const { return buffer_.iteratorLowerBound(ts); }
    
private:
    // Buffer for incoming lidar points
    PointBufferType buffer_; // NB: Points are buffered in body-frame!
    inline bool acceptPoint(double ts, float x, float y, float z, float i) const;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    // Extrinsic calibration (lidar->body)
    Eigen::Matrix3f bRl_;
    Eigen::Vector3f btl_;
};
