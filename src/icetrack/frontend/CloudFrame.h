#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

#include "utils/pointcloud.h"

/*
This is the main class of a CloudFrame, responsible to track lidar points, stamps, etc. in a given time frame (t0, t1).
*/
class CloudFrame{
public:
    // Construct with dynamic capacity
    CloudFrame(int idx, size_t capacity)
        : idx_(idx), size_(0),
        p_local_(3, capacity), p_global_(3, capacity),
        intensities_(capacity), timestamps_(capacity) {}

    // Interface
    void addPoint(const Eigen::Vector3f& pos, const uint8_t i, const double ts){
        if (full()){
            ROS_ERROR("addPoint() - CloudFrame is full");
            return;
        }

        p_local_.col(size_) = pos;
        intensities_(size_) = i;
        timestamps_(size_) = ts;

        ++size_;
    }

    void transformPoints(const Eigen::Matrix4f& T){
        const Eigen::Matrix3f& R = T.block<3, 3>(0, 0);
        const Eigen::Vector3f& t = T.block<3, 1>(0, 3);

        p_global_ = (R * p_local_).colwise() + t;
    }

    // Accessors
    PointCloudPtr localCloud() const{
        return EigenToPointCloudPtr(p_local_);
    }
    PointCloudPtr globalCloud() const{
        return EigenToPointCloudPtr(p_global_);
    }
    
    size_t size() const { return size_; }
    size_t capacity() const { return p_local_.cols(); }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == capacity(); }

    int idx() const { return idx_; }
    double t0() const { return empty() ? 0.0:  timestamps_(0); }
    double t1() const { return empty() ? 0.0:  timestamps_(size_-1); }

private:
    int idx_;                                   // Idx of local frame in posegraph
    size_t size_;                               // Number of points
    Eigen::Matrix3Xf p_local_;                  // Local frame vectors
    Eigen::Matrix3Xf p_global_;                 // Nav frame vectors
    Eigen::Matrix<uint8_t, 1, -1> intensities_; // Lidar intensity
    Eigen::Matrix<double, 1, -1> timestamps_;   // Timestamps
};