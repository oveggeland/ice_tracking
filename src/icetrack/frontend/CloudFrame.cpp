#include "frontend/CloudFrame.h"

// Constructor for shallow copy slice
CloudFrame::CloudFrame(const CloudFrame& other, size_t idx0, size_t idx1)
    : idx_(other.idx_), size_(idx1 - idx0),
    p_local_(other.p_local_.block(0, idx0, 3, size_)),
    p_global_(other.p_global_.block(0, idx0, 3, size_)),
    intensities_(other.intensities_.block(0, idx0, 1, size_)),
    timestamps_(other.timestamps_.block(0, idx0, 1, size_)) {
}


CloudFrame::CloudFrame(int idx, size_t capacity)
    : idx_(idx), size_(0), 
    p_local_(3, capacity), p_global_(3, capacity),
    intensities_(capacity), timestamps_(capacity) {
}

void CloudFrame::addPoint(const Eigen::Vector3f& pos, const float i, const double ts) {
    if (full()) {
        ROS_ERROR("addPoint() - CloudFrame is full"); // This should never happen
        return;
    }

    p_local_.col(size()) = pos;
    intensities_(size()) = i;
    timestamps_(size()) = ts;

    ++size_;
}

void CloudFrame::merge(const CloudFrame& other, bool copyLocal, bool copyGlobal, bool copyIntensities, bool copyTimestamps) {
    if (size() + other.size() > capacity()) {
        ROS_ERROR("merge() - CloudFrame is full"); // This should never happen
        return;
    }

    if (copyLocal) 
        p_local_.block(0, size(), 3, other.size()) = other.p_local_;
    if (copyGlobal)
        p_global_.block(0, size(), 3, other.size()) = other.p_global_;
    if (copyIntensities)
        intensities_.block(0, size(), 1, other.size()) = other.intensities_;
    if (copyTimestamps)
        timestamps_.block(0, size(), 1, other.size()) = other.timestamps_;

    size_ += other.size();
}

void CloudFrame::transformPoints(const Eigen::Matrix4f& T) {
    const Eigen::Matrix3f& R = T.block<3, 3>(0, 0);
    const Eigen::Vector3f& t = T.block<3, 1>(0, 3);

    p_global_ = (R * p_local_).colwise() + t;
}

// Binary search for lower bound
int CloudFrame::lowerBound(double ts) const {
    if (ts < t0())
        return 0;
    else if (ts > t1())
        return size();

    int left = 0;
    int right = size() - 1;

    while (left < right) {
        int mid = left + (right - left) / 2;
        if (timestamps_(mid) < ts) {
            left = mid + 1;
        } else {
            right = mid;
        }
    }

    return left;
}


/*
Return a tensor cloud with global position and intensities
*/
TensorCloud CloudFrame::toCloud() const {
    const open3d::core::Tensor positions = EigenToTensorFloat(p_global_);
    const open3d::core::Tensor intensities = EigenToTensorFloat(intensities_);

    open3d::t::geometry::PointCloud cloud(positions);
    cloud.SetPointAttr("intensities", intensities);


    return cloud;
}