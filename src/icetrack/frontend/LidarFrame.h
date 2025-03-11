#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/geometry/PointCloud.h>
#include <open3d/visualization/utility/DrawGeometry.h>

#include "PointBuffer.h" // For point definition

/*
LidarFrame is the set of points acquired between pose0 and pose1. 
*/
class LidarFrame {
public:
    // Constructor
    LidarFrame(const int frame_id, const double ts, const size_t capacity);

    // Point handling
    void addPoint(const PointXYZIT& point);
    
    void downSample(const double voxel_size);
    void undistort(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1);

    void show(const bool distorted=true){
        std::shared_ptr<open3d::geometry::PointCloud> cloud = distorted ? distorted_ : undistorted_;
        open3d::visualization::DrawGeometries({cloud}, "LidarFrame Viewer");
    }

    // Accessors
    int id() const { return id_; }
    double timestamp() const { return ts_; }

    size_t size() const { return distorted_->points_.size(); }

    std::shared_ptr<const open3d::geometry::PointCloud> distorted() const { return distorted_; }
    std::shared_ptr<const open3d::geometry::PointCloud> undistorted() const { return undistorted_; }

    const std::vector<float>& intensities() const { return intensities_; }
    const std::vector<float>& dt() const { return dt_; }

private:
    int id_;        // Frame id - corresponds to a state idx in pose graph
    double ts_;     // Timestamp of frame

    // Keep track of both distorted and undistorted cloud (undistortion might be re-applied)
    std::shared_ptr<open3d::geometry::PointCloud> distorted_ = nullptr;
    std::shared_ptr<open3d::geometry::PointCloud> undistorted_ = nullptr;

    // Point cloud attributes
    std::vector<float> intensities_;    // Lidar intensity 
    std::vector<float> dt_;             // Time delta to ts_
};