#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include "utils/ros_params.h"
#include "utils/file_system.h"

class CloudProcessor{
public:
    CloudProcessor(const ros::NodeHandle& nh);

    // Key entry point. Applies processing pipeline to incoming pcd. 
    void processCloud(double ts, open3d::t::geometry::PointCloud& pcd) const;

private:
    // Processing parameters
    double grid_size_;
    double smoothing_radius_;
    double deformation_radius_;

    // Pipeline functionality
    void gridDownSample(open3d::t::geometry::PointCloud& pcd) const;
    void smoothCloud(open3d::t::geometry::PointCloud& pcd) const;
    void estimateLocalDeformation(open3d::t::geometry::PointCloud& pcd) const;
    void estimatePlaneDeviations(open3d::t::geometry::PointCloud& pcd) const;

    // Output
    bool save_clouds_;
    std::string cloud_path_;

    void saveCloud(double ts, const open3d::t::geometry::PointCloud& pcd) const;
};