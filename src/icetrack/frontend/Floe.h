#pragma once

#include <random>
#include <ros/ros.h>

#include <open3d/geometry/PointCloud.h>
#include "RasterizedCluster.h"

class Floe {
public:
    // Default constructor
    Floe() : floe_id_(-1), cloud_(nullptr) {}

    // Constructor with ID
    Floe(int id) : floe_id_(id), cloud_(std::make_shared<open3d::geometry::PointCloud>()) {
        assignRandomColor();
    }

    // Constructor with ID and capacity
    Floe(int id, int capacity) : Floe(id) {
        reserve(capacity);
    }

    // Estimate area of intersection between this and other
    double intersection(const Floe& other) const;
    double getArea() const { return Raster(cloud_->points_).occupiedArea(); }

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

    void setColor(const Eigen::Vector3d& color){
        color_ = color;
    }

    std::vector<int> associatePoints(const std::vector<Eigen::Vector3d>& points);

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
                cloud_->colors_[new_size] = cloud_->colors_[i];
                frame_id_[new_size] = frame_id_[i];
                frame_idx_[new_size] = frame_idx_[i];
                ++new_size;
            }
        }
    
        // Resize the vectors to the new size
        cloud_->points_.resize(new_size);
        cloud_->colors_.resize(new_size);
        frame_id_.resize(new_size);
        frame_idx_.resize(new_size);
    }

    // Accessors
    int size() const { return cloud_->points_.size(); }
    int id() const { return floe_id_; }
    std::shared_ptr<open3d::geometry::PointCloud> getCloud() const { return cloud_; }
    
    
    // Add a point to the Floe and assign the default color
    void addPoint(const Eigen::Vector3d& point, const int frame_id, const int frame_idx) {
        cloud_->points_.push_back(point);    
        cloud_->colors_.push_back(color_);  // Assign the floe's color
        frame_id_.push_back(frame_id);
        frame_idx_.push_back(frame_idx);
    }

    // Helper to copy a point from another Floe, maintaining the same color
    void copyFrom(const Floe& source, const int idx){
        addPoint(
            source.cloud_->points_[idx],
            source.frame_id_[idx],
            source.frame_idx_[idx]
        );
    }  

    // Assign a random color at creation
    void assignRandomColor() {
        static std::random_device rd;
        static std::mt19937 rng(rd());
        static std::uniform_real_distribution<double> dist(0.0, 1.0); // Open3D uses normalized colors [0,1]

        color_ = Eigen::Vector3d(dist(rng), dist(rng), dist(rng)); 
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> getBoundingBox(){
        return {
            cloud_->GetMinBound(),
            cloud_->GetMaxBound()
        };
    }

    std::vector<int> findOutliers();

    int getFrameId(const int point_idx) const { return frame_id_[point_idx]; }
    int getFrameIdx(const int point_idx) const { return frame_idx_[point_idx]; }

private:
    int floe_id_;  // ID of this floe


    // Stores the points for this floe
    std::shared_ptr<open3d::geometry::PointCloud> cloud_;

    // Metadata about the origin of points
    std::vector<int> frame_id_;   // ID of the frame the point came from
    std::vector<int> frame_idx_;  // Index of the point within the frame

    Eigen::Vector3d color_;  // Assigned random color for this floe
};
