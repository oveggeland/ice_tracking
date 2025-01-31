#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/StampedRingBuffer.h"


// Some useful naming
using PointCloud = open3d::geometry::PointCloud;
using PointCloudSharedPtr = std::shared_ptr<PointCloud>;

using FrameBuffer = std::map<int, PointCloudSharedPtr>;

using PointType = PointXYZT;
using PointBuffer = StampedRingBuffer<PointType>;
using PointBufferIterator = StampedRingBufferIterator<PointType>;


/**
 * Main class for maintenance of Lidar points. We adopt two types of buffers:
 * 
 * 1. The PointBuffer: A ringbuffer that is used to store only the most recent raw LiDAR
 * data. 
 * 
 * 2. The FrameBuffer: A std::map, where the key corresponds to a state idx and the element
 * is a cloud containing all points captured since the previous state idx. The cloud is 
 * represented in the IMU frame at the state idx. 
 */
class LidarBuffer{
public: 
    LidarBuffer(const ros::NodeHandle& nh);

    // Modify and access point buffer
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
    const PointBufferIterator lowerBoundPointIterator(double ts) const { return point_buffer_.iteratorLowerBound(ts); }

    // Modify and access frame buffer
    void createFrame(int state_idx, double t0, double t1, const Pose3& pose0, const Pose3& pose1);
    PointCloudSharedPtr getFrame(int idx) const;

    // Const accesors for buffers
    const PointBuffer& getPointBuffer() const { return point_buffer_; }
    const FrameBuffer& getFrameBuffer() const { return frame_buffer_; }

private:
    // Buffers
    PointBuffer point_buffer_;
    FrameBuffer frame_buffer_;

    // Calibration matrix (lidar->imu)
    Pose3 bTl_;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;
};