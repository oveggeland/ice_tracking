#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "utils/pointcloud.h"

using CloudPositionType = Eigen::Matrix3Xf;
using CloudIntensityType = std::vector<uint8_t>;
using CloudTimestampType = std::vector<double>;

class CloudFrame {
public:
    using Ptr = std::shared_ptr<CloudFrame>;

    CloudFrame(int idx, size_t capacity);

    void addPoint(const Eigen::Vector3f& pos, const uint8_t i, const double ts);
    void merge(const CloudFrame& other, bool copyLocal = true, bool copyGlobal = true, bool copyIntensities = true, bool copyTimestamps = true);
    void transformPoints(const Eigen::Matrix4f& T);
    int lowerBound(double ts) const;
    Eigen::Matrix3Xf getPointsWithin(double t0, double t1) const;

    PointCloudPtr localCloud() const;
    PointCloudPtr globalCloud() const;

    inline size_t size() const { return size_; }
    inline size_t capacity() const { return p_local_.cols(); }
    inline bool empty() const { return size() == 0; }
    inline bool full() const { return size() == capacity(); }

    inline int idx() const { return idx_; }
    inline double t0() const { return empty() ? 0.0 : timestamps_.front(); }
    inline double t1() const { return empty() ? 0.0 : timestamps_.back(); }

    inline const CloudPositionType& local() const { return p_local_; }
    inline const CloudPositionType& global() const { return p_global_; }
    inline const CloudIntensityType& intensities() const { return intensities_; }
    inline const CloudTimestampType& timestamps() const { return timestamps_; }

private:
    size_t size_;
    int idx_;
    CloudPositionType p_local_;
    CloudPositionType p_global_;
    CloudIntensityType intensities_;
    CloudTimestampType timestamps_;
};