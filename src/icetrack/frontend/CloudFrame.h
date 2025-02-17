#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

#include "utils/pointcloud.h"

using CloudPositionType = Eigen::Matrix3Xf;
using CloudIntensityType = std::vector<uint8_t>;
using CloudTimestampType = std::vector<double>;

/*
This is the main class of a CloudFrame, responsible to track lidar points, stamps, etc. in a given time frame (t0, t1).
*/
class CloudFrame{
public:
    // Construct with dynamic capacity
    CloudFrame(int idx, size_t capacity)
        : idx_(idx), p_local_(3, capacity), p_global_(3, capacity){
            intensities_.reserve(capacity);
            timestamps_.reserve(capacity);
        }

    // Interface
    void addPoint(const Eigen::Vector3f& pos, const uint8_t i, const double ts){
        if (full()){
            ROS_ERROR("addPoint() - CloudFrame is full");
            return;
        }

        p_local_.col(size()) = pos;
        intensities_.push_back(i);
        timestamps_.push_back(ts);
    }

    void transformPoints(const Eigen::Matrix4f& T){
        const Eigen::Matrix3f& R = T.block<3, 3>(0, 0);
        const Eigen::Vector3f& t = T.block<3, 1>(0, 3);

        p_global_ = (R * p_local_).colwise() + t;
    }


    int lowerBound(double ts) const {
        // Handle edge cases manually for efficiency
        if (ts < t0())
            return 0;
        else if (ts > t1())
            return size();

        // Binary search
        auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), ts);
        return std::distance(timestamps_.begin(), it);
    }
    
    Eigen::Matrix3Xf getPointsWithin(double t0, double t1) const{
        int idx0 = lowerBound(t0);
        int idx1 = lowerBound(t1);
        return p_global_.block(0, idx0, 3, idx1-idx0);
    }

    // Accessors
    PointCloudPtr localCloud() const{
        return EigenToPointCloudPtr(p_local_);
    }
    PointCloudPtr globalCloud() const{
        return EigenToPointCloudPtr(p_global_);
    }

    size_t size() const { return timestamps_.size(); }
    size_t capacity() const { return p_local_.cols(); }
    bool empty() const { return size() == 0; }
    bool full() const { return size() == capacity(); }

    int idx() const { return idx_; }
    double t0() const { return empty() ? 0.0:  timestamps_.front(); }
    double t1() const { return empty() ? 0.0:  timestamps_.back(); }

    const CloudPositionType& local() const { return p_local_; }
    const CloudPositionType& global() const { return p_global_; }
    const CloudIntensityType& intensities() const { return intensities_; }
    const CloudTimestampType& timestamps() const { return timestamps_; }
private:
    int idx_;                                       // Idx of local frame in posegraph
    CloudPositionType p_local_;                     // Local frame vectors
    CloudPositionType p_global_;                    // Nav frame vectors
    CloudIntensityType intensities_;                // Lidar intensity
    CloudTimestampType timestamps_;                 // Timestamps
};