#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <yaml-cpp/yaml.h>

#include "icetrack/common.h"
#include "icetrack/StampedRingBuffer.h"

struct PointXYZI{
    float x, y, z, i;
};

class Lidar{
public: 
    Lidar();
    Lidar(ros::NodeHandle nh);

    double getPointInterval() const {return point_interval_;}
    double getMinDistance() const {return max_dist_;}
    double getMaxDistance() const {return max_dist_;}

    std::shared_ptr<StampedRingBuffer<PointXYZI>> getPointBuffer() const{
        return point_buffer_;
    }

    void addFrame(sensor_msgs::PointCloud2::ConstPtr msg);

private:
    ros::NodeHandle nh_;

    double ts_head_;
    std::shared_ptr<StampedRingBuffer<PointXYZI>> point_buffer_;

    double min_intensity_;
    double min_dist_;
    double max_dist_;

    double point_interval_;    // Temporal distance between measurements (We use half the interval because of dual return mode)
};