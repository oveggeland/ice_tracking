#pragma once

#include <open3d/geometry/PointCloud.h>

class Floe {
public:
    // Default constructor
    Floe() : floe_id_(-1), cloud_(nullptr) {}

    // Constructor with ID
    Floe(int id) : floe_id_(id), cloud_(std::make_shared<open3d::geometry::PointCloud>()) {}

    // Constructor with ID and capacity
    Floe(int id, int capacity) : Floe(id) {
        reserve(capacity);
    }

    // Clear the point cloud and associated metadata
    void clear() {
        cloud_->Clear();  // Clear the Open3D point cloud
        frame_id_.clear();
        frame_idx_.clear();
    }

    // Reserve space for the point cloud and metadata vectors
    void reserve(const int n_points) {
        cloud_->points_.reserve(n_points);  // Reserve memory for points
        frame_id_.reserve(n_points);
        frame_idx_.reserve(n_points);
    }

    // Set a constant color for the point cloud in the Floe
    void setColor(const Eigen::Vector3d& color) {
        cloud_->colors_.resize(cloud_->points_.size(), color);  // Set color for all points in the cloud
    }
 
    // Accessors
    int size() const { return cloud_->points_.size(); }
    int id() const { return floe_id_; }
    std::shared_ptr<open3d::geometry::PointCloud> getCloud() const { return cloud_; }

    // Add a point to the Floe
    void addPoint(const Eigen::Vector3d& point, const int frame_id, const int frame_idx) {
        cloud_->points_.push_back(point);    // Add point to the point cloud
        frame_id_.push_back(frame_id);
        frame_idx_.push_back(frame_idx);
    }

    // Helper to copy a point from another flow
    void copyFrom(const Floe& source, const int idx){
        cloud_->points_.push_back(source.cloud_->points_[idx]);
        frame_id_.push_back(source.frame_id_[idx]);
        frame_idx_.push_back(source.frame_idx_[idx]);
    }

    /////////////////////// Members //////////////////////////////////7
    int floe_id_;  // ID of this floe

    // Stores the points for this floe
    std::shared_ptr<open3d::geometry::PointCloud> cloud_;

    // Metadata about the origin of points
    std::vector<int> frame_id_;   // ID of the frame the point came from
    std::vector<int> frame_idx_;  // Index of the point within the frame
};
