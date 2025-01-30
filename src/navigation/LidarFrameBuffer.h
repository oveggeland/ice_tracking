#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"
#include "navigation/LidarPointBuffer.h"

#include "utils/calibration.h"

using PointCloud = open3d::geometry::PointCloud;
using PointCloudSharedPtr = std::shared_ptr<PointCloud>;

class LidarFrameBuffer{
public:
    LidarFrameBuffer(const ros::NodeHandle& nh, const LidarPointBuffer& point_buffer);

    void createFrame(int idx, double t0, double t1, const Pose3& pose0, const Pose3& pose1);
    PointCloudSharedPtr getFrame(int idx) const;

private:
    Pose3 bTl_;
    
    // LidarPointBuffer
    const LidarPointBuffer& point_buffer_;

    // Frame map (bookkeeping of undistorted lidar frames)
    std::map<int, PointCloudSharedPtr> frame_buffer_;
};