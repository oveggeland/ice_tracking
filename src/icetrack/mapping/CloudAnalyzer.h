/*
Library for point cloud analysis. Retrieve statistics, estimtae freeboard, deformation, etc.
*/
#pragma once

#include <fstream>

#include <ros/ros.h>

#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include "utils/file_system.h"
#include "utils/ros_params.h"

// This struct contains the information we want to extract from the point cloud
struct CloudStatistics{
    int count;
    float grid_size; // Square meters
    float elevation_moments[2];
    float intensity_moments[2];
    float freeboard;
    float deformation;
};


class CloudAnalyzer{
public:
    CloudAnalyzer(const ros::NodeHandle& nh);

    // Key entry point. Takes a pcd and generates statistics about it.
    void analyzeCloud(double ts, const open3d::t::geometry::PointCloud& pcd);
private:
    // Functions to estimate freeboard and deformation of pointclouds
    float estimateFreeboard(const open3d::t::geometry::PointCloud& pcd) const;
    float estimateDeformation(const open3d::t::geometry::PointCloud& pcd) const;

    // Save
    bool save_stats_;
    std::ofstream f_out_;

    void saveStats(double ts, const CloudStatistics& stats);
};