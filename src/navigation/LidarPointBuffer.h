#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "utils/ros_params.h"
#include "utils/StampedRingBuffer.h"

class LidarPointBuffer{
public: 
    LidarPointBuffer(const ros::NodeHandle& nh);

    // Interface
    void addLidarPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

    StampedRingBufferIterator<PointXYZT> lowerBound(double ts) const { return point_buffer_.iteratorLowerBound(ts); }
private:
    StampedRingBuffer<PointXYZT> point_buffer_;

    // Lidar parameters
    double ts_head_ = 0.0;
    double point_interval_;

    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;
};