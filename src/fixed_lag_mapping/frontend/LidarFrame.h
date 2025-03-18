#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <open3d/geometry/PointCloud.h>

#include "point_types.h" // For point definition

/*
LidarFrame is the set of points acquired between pose0 and pose1. 
*/
class LidarFrame {
public:
    // Constructor
    LidarFrame(const int frame_id, const double ts, const size_t capacity);

    // Point handling
    void addPoint(const PointXYZIT& point);
    void undistort(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1);
    void undistortFirstOrder(const double t0, const double t1, const gtsam::Pose3& pose0, const gtsam::Pose3& pose1);

    // Generate open3d::geometry::pointcloud from frame
    std::shared_ptr<open3d::geometry::PointCloud> toCloud(bool undistorted=true) const;

    // Accessors
    int id() const { return id_; }
    double timestamp() const { return ts_; }

    size_t size() const { return distorted_points_.size(); }

    const std::vector<Eigen::Vector3f>& distortedPoints() const { return distorted_points_; }
    const std::vector<Eigen::Vector3f>& undistortedPoints() const { return undistorted_points_; }
    const std::vector<float>& intensities() const { return intensities_; }
    const std::vector<float>& dt() const { return dt_; }

private:
    int id_;        // Frame id - corresponds to a state idx in pose graph
    double ts_;     // Timestamp of frame

    // Keep track of both distorted and undistorted points
    std::vector<Eigen::Vector3f> distorted_points_;
    std::vector<Eigen::Vector3f> undistorted_points_;

    // Point cloud attributes
    std::vector<float> intensities_;    // Lidar intensity 
    std::vector<float> dt_;             // Time delta to ts_
};