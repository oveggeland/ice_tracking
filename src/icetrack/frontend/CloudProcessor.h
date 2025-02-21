#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/t/geometry/PointCloud.h>

#include "frontend/CloudRaster.h"

#include "utils/ros_params.h"
#include "utils/file_system.h"

class CloudProcessor{
public:
    CloudProcessor(const ros::NodeHandle& nh);

    open3d::t::geometry::PointCloud processCloud(const open3d::t::geometry::PointCloud& pcd);
private:
    // Processing parameters
    double grid_size_;
    double smoothing_window_size_;
    double deformation_window_size_;

    // Output
    bool save_clouds_;
    std::string cloud_path_;

    void saveCloud(const open3d::t::geometry::PointCloud& pcd) const;
};