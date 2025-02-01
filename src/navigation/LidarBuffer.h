#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "utils/ros_params.h"
#include "utils/StampedRingBuffer.h"

using PointType = PointXYZT;
using PointBuffer = StampedRingBuffer<PointType>;
using PointBufferIterator = StampedRingBufferIterator<PointType>;

class LidarBuffer{
public: 
    LidarBuffer(const ros::NodeHandle& nh);

    // Modify and access point buffer
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
    const PointBuffer& pointBuffer() const { return buffer_; }

private:
    PointBuffer buffer_;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;
};