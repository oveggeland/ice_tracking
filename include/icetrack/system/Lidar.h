#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "icetrack/utils/ros_params.h"
#include "icetrack/utils/StampedRingBuffer.h"

struct RawLidarPoint {
    double ts;
    float x, y, z, i;
};

class Lidar{
public: 
    Lidar(ros::NodeHandle nh);

    bool newMessage(sensor_msgs::PointCloud2::ConstPtr msg);

    double getPointInterval() const { return point_interval_; }
    double getMinDistance() const { return sqrt(min_dist_squared_); }
    double getMaxDistance() const { return sqrt(max_dist_squared_); }
    const StampedRingBuffer<RawLidarPoint>& pointBuffer() const { return point_buffer_; }

private:
    double ts_head_ = 0.0;
    StampedRingBuffer<RawLidarPoint> point_buffer_;

    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    double point_interval_;
};