#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "icetrack/utils/utils.h"
#include "icetrack/utils/StampedRingBuffer.h"

struct RawLidarPoint {
    double ts;
    float x, y, z, i;
};

class Lidar{
public: 
    Lidar();
    Lidar(ros::NodeHandle nh);

    double getPointInterval() const;
    double getMinDistance() const;
    double getMaxDistance() const;

    std::shared_ptr<StampedRingBuffer<RawLidarPoint>> getConstBufferPointer() const;

    bool newMessage(sensor_msgs::PointCloud2::ConstPtr msg);

private:
    double ts_head_ = 0.0;
    std::shared_ptr<StampedRingBuffer<RawLidarPoint>> point_buffer_;

    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    double point_interval_;
};