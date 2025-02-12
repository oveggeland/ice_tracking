#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "frontend/FrameBuffer.h"

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "utils/pointcloud.h"

class Map{
public:
    Map(ros::NodeHandle& nh);

    void reset(size_t size);
    void addFrame(const Eigen::Matrix4f& T, const FrameType& frame);

    void visualizeMap() const;
    void publishAsPointCloud2();

    // Accessors
    size_t size() const { return size_; }
    size_t capacity() const { return positions_.cols(); }
private:
    size_t size_ = 0;
    Eigen::Matrix3Xf positions_;
    Eigen::Matrix<uint8_t, 1, -1> intensities_;
    Eigen::Matrix<double, 1, -1> timestamps_;

    // Publishing
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_msg_;

    void initializeCloudMsg();
};

