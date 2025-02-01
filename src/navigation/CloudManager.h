#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

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

class CloudManager{
public: 
    CloudManager(const ros::NodeHandle& nh, const gtsam::BatchFixedLagSmoother& smoother);

    // Modify and access point buffer
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Modify and access frame buffer
    void createFrame(int state_idx);

    const FrameBuffer& frameBuffer() const { return frame_buffer_; }
    const PointBuffer& pointBuffer() const { return point_buffer_; }

private:
    // Buffers
    PointBuffer point_buffer_;
    FrameBuffer frame_buffer_;

    // Reference to pose smoother
    const gtsam::BatchFixedLagSmoother& smoother_;

    // Calibration matrix (lidar->imu)
    gtsam::Pose3 bTl_;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;
};