#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"
#include "navigation/LidarBuffer.h"

struct OdometryFrame{
    Pose3 pose;
    std::shared_ptr<open3d::geometry::PointCloud> pcd;
};

class LidarOdometry{
public:
    LidarOdometry(const ros::NodeHandle& nh, const LidarBuffer& lidar_buffer);

private:
    const LidarBuffer& lidar_buffer_;

    // Config
    int min_point_count_ = 10000;              // Minimum number of points per frame
    int frame_interval_ = 2;            // interval between frames to be aligned
    double icp_threshold_ = 2.0;

    bool estimateOdometry(int idx1);
};