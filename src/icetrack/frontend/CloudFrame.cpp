#include "frontend/CloudFrame.h"

CloudFrame::CloudFrame(int idx, size_t capacity)
    : idx_(idx), size_(0), p_local_(3, capacity), p_global_(3, capacity) {
    intensities_.reserve(capacity);
    timestamps_.reserve(capacity);
}

void CloudFrame::addPoint(const Eigen::Vector3f& pos, const uint8_t i, const double ts) {
    if (full()) {
        ROS_ERROR("addPoint() - CloudFrame is full"); // This should never happen
        return;
    }

    p_local_.col(size()) = pos;
    intensities_.push_back(i);
    timestamps_.push_back(ts);

    ++size_;
}

void CloudFrame::merge(const CloudFrame& other, bool copyLocal, bool copyGlobal, bool copyIntensities, bool copyTimestamps) {
    if (size() + other.size() > capacity()) {
        ROS_ERROR("merge() - CloudFrame is full"); // This should never happen
        return;
    }

    if (copyLocal) {
        p_local_.block(0, size(), 3, other.size()) = other.local();
    }
    if (copyGlobal) {
        p_global_.block(0, size(), 3, other.size()) = other.global();
    }
    if (copyIntensities) {
        intensities_.insert(intensities_.end(), other.intensities().begin(), other.intensities().end());
    }
    if (copyTimestamps) {
        timestamps_.insert(timestamps_.end(), other.timestamps().begin(), other.timestamps().end());
    }

    size_ += other.size();
}

void CloudFrame::transformPoints(const Eigen::Matrix4f& T) {
    const Eigen::Matrix3f& R = T.block<3, 3>(0, 0);
    const Eigen::Vector3f& t = T.block<3, 1>(0, 3);

    p_global_ = (R * p_local_).colwise() + t;
}

int CloudFrame::lowerBound(double ts) const {
    if (ts < t0())
        return 0;
    else if (ts > t1())
        return size();

    auto it = std::lower_bound(timestamps_.begin(), timestamps_.end(), ts);
    return std::distance(timestamps_.begin(), it);
}

Eigen::Matrix3Xf CloudFrame::getPointsWithin(double t0, double t1) const {
    int idx0 = lowerBound(t0);
    int idx1 = lowerBound(t1);
    return p_global_.block(0, idx0, 3, idx1 - idx0);
}

PointCloudPtr CloudFrame::localCloud() const {
    return EigenToPointCloudPtr(p_local_);
}

PointCloudPtr CloudFrame::globalCloud() const {
    return EigenToPointCloudPtr(p_global_);
}
