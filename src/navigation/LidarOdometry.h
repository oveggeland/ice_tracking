#pragma once

#include <ros/ros.h>

#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"
#include "navigation/LidarBuffer.h"

#include "utils/calibration.h"
#include "utils/StampedRingBuffer.h"

class LidarOdometry{
public:
    LidarOdometry(const ros::NodeHandle& nh, const LidarBuffer& lidar_buffer);

    void addFrame(int state_idx, double t0, double t1, Pose3 pose0, Pose3 pose1);

private:
    const StampedRingBuffer<PointXYZT>& point_buffer_;

    // Calib
    Pose3 bTl_;

    // Extract cloud (in body frame) between t0 and t1 by interpolating the input poses
    open3d::geometry::PointCloud createPointCloud(double t0, double t1, const Pose3& pose0, const Pose3& pose1) const;
};