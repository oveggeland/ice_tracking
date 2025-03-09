#pragma once

#include <ros/ros.h>

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>

#include "RasterizedCluster.h"

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
        point_class_.clear();
        tree_ = nullptr;
    }

    void buildSearchTree(){
        tree_ = std::make_shared<open3d::geometry::KDTreeFlann>(*cloud_);
    }

    // Reserve space for the point cloud and metadata vectors
    void reserve(const int n_points) {
        cloud_->points_.reserve(n_points);  // Reserve memory for points
        frame_id_.reserve(n_points);
        frame_idx_.reserve(n_points);
        point_class_.reserve(n_points);
    }

    // Get current capacity
    int capacity() const {
        return cloud_->points_.capacity();
    }

    // Make sure we have room for n_points more than current size
    void reserveAdditional(const int n_points){
        const int required_cap = size() + n_points;
        if (required_cap > capacity())
            reserve(required_cap);
    }

    void removeByIndex(const std::vector<int>& indices) {
        if (indices.empty()) return;
    
        // Create a mask to mark points for removal
        std::vector<bool> remove_mask(size(), false);
        for (const int& idx : indices)
            remove_mask[idx] = true;
    
        // Compact the vectors by keeping only the points not marked for removal
        int new_size = 0;
        for (int i = 0; i < size(); ++i) {
            if (!remove_mask[i]) {
                cloud_->points_[new_size] = cloud_->points_[i];
                frame_id_[new_size] = frame_id_[i];
                frame_idx_[new_size] = frame_idx_[i];
                point_class_[new_size] = point_class_[i];
                ++new_size;
            }
        }
    
        // Resize the vectors to the new size
        cloud_->points_.resize(new_size);
        frame_id_.resize(new_size);
        frame_idx_.resize(new_size);
        point_class_.resize(new_size);
    
        // Reset the KDTree
        tree_ = nullptr;
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
        point_class_.push_back(0); // Class defaults to noise
    }

    // Helper to copy a point from another flow
    void copyFrom(const Floe& source, const int idx){
        addPoint(
            source.cloud_->points_[idx],
            source.frame_id_[idx],
            source.frame_idx_[idx]
        );
    }   

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> getBoundingBox(){
        return {
            cloud_->GetMinBound(),
            cloud_->GetMaxBound()
        };
    }

    std::vector<int> findOutliers();

    bool isCompatible(const Eigen::Vector3d& point){
        if (!tree_){
            ROS_WARN("No searc tree available for compatibility check");
            buildSearchTree();
        }
        
        std::vector<int> idx;
        std::vector<double> r2;

        tree_->SearchKNN(point, 10, idx, r2);
        if (*std::max_element(r2.begin(), r2.end()) < 1)
            return true;
        return false;
    }

    /////////////////////// Members //////////////////////////////////7
    int floe_id_;  // ID of this floe

    // Stores the points for this floe
    std::shared_ptr<open3d::geometry::PointCloud> cloud_;
    std::vector<uint8_t> point_class_;     // 0: noise, 1: edge, 2: core 

    // Efficient lookup
    std::shared_ptr<open3d::geometry::KDTreeFlann> tree_ = nullptr;

    // Metadata about the origin of points
    std::vector<int> frame_id_;   // ID of the frame the point came from
    std::vector<int> frame_idx_;  // Index of the point within the frame
};
