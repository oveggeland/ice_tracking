#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "utils/pointcloud.h"

class CloudFrame {
public:
    using Ptr = std::shared_ptr<CloudFrame>;

    CloudFrame(int idx, size_t capacity);   // Constructor

    void addPoint(const Eigen::Vector3d& pos, const float i, const double ts);
    void setTransform(const Eigen::Matrix4d& T) { transform_ = T; }

    void setFloeLabel(const int idx, const int label){ floe_labels_[idx] = label; }

    void downSample(double voxel_size);

    //const TensorCloud toTensorCloud() const;
    void show() const;

    // Accessors
    inline int size() const { return cloud_.points_.size(); }
    inline size_t capacity() const { return cloud_.points_.capacity(); }
    inline bool empty() const { return size() == 0; }
    inline bool full() const { return size() == capacity(); }

    inline int idx() const { return idx_; }
    inline double t0() const { return empty() ? -1 : timestamps_.front(); }
    inline double t1() const { return empty() ? -1 : timestamps_.back(); }

    inline const Eigen::Matrix4d& transform() const { return transform_; }
    inline const std::vector<Eigen::Vector3d>& positions() const { return cloud_.points_; }
    inline const std::vector<float>& intensities() const { return intensities_; }
    inline const std::vector<double>& timestamps() const { return timestamps_; }
    inline const std::vector<int>& floeLabels() const { return floe_labels_; }

    inline const open3d::geometry::PointCloud& local() const { return cloud_; }
    std::shared_ptr<open3d::geometry::PointCloud> global() const;

private:
    int idx_;
    Eigen::Matrix4d transform_;
    open3d::geometry::PointCloud cloud_;
    std::vector<float> intensities_;
    std::vector<double> timestamps_;
    std::vector<int> floe_labels_;
};