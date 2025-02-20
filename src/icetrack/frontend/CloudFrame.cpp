#include "frontend/CloudFrame.h"


CloudFrame::CloudFrame(int idx, size_t capacity) : idx_(idx){
    cloud_.points_.reserve(capacity);
    intensities_.reserve(capacity);
    timestamps_.reserve(capacity);
}

void CloudFrame::addPoint(const Eigen::Vector3d& pos, const float i, const double ts) {
    if (full()) {
        ROS_ERROR("addPoint() - CloudFrame is full"); // This should never happen
        return;
    }

    cloud_.points_.push_back(pos);
    intensities_.push_back(i);
    timestamps_.push_back(ts);
}



void CloudFrame::downSample(double voxel_size) {
    auto [cloud_ptr, traces, inliers] = cloud_.VoxelDownSampleAndTrace(voxel_size, cloud_.GetMinBound(), cloud_.GetMaxBound(), false); // TODO: Use approximate class?

    // Allocate downsampled vectors
    std::vector<float> intensities_ds;
    std::vector<double> timestamps_ds;
    intensities_ds.reserve(inliers.size());
    timestamps_ds.reserve(inliers.size());

    for (int i = 0; i < inliers.size(); ++i){
        float i_sum = 0;
        double t_sum = 0;
        int cnt = 0;
        for (const auto& idx: inliers[i]){
            i_sum += intensities_[idx];
            t_sum += timestamps_[idx];
            ++cnt;
        }
        intensities_ds.push_back(i_sum / cnt);
        timestamps_ds.push_back(t_sum / cnt);
    }

    // Update vectors
    cloud_ = *cloud_ptr;
    intensities_ = intensities_ds;
    timestamps_ = timestamps_ds;
}


void CloudFrame::show() const{
    visualizeCloud(cloud_);
}

std::shared_ptr<open3d::geometry::PointCloud> CloudFrame::global() const {
    auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(cloud_.points_);
    cloud_ptr->Transform(transform_);
    return cloud_ptr;
}


/*
Return a tensor cloud with global position and intensities
*/
// const TensorCloud CloudFrame::toTensorCloud() const {
//     std::vector<float> flat_positions;
//     std::vector<float> flat_intensitites;
//     flat_positions.reserve(3*size());
//     flat_intensities.reserve(size());

//     for (int i = 0; i < size(); i++){
//         Eigen::Vector3d pos = 
//         flat_positions.push_back()
//     }

//     const open3d::core::Tensor positions = open3d::core::Tensor(static_cast<void*>(cloud_.points_.data()), open3d::core::Dtype::Float32, {size(), 3});
//     const open3d::core::Tensor intensities = open3d::core::Tensor(static_cast<void*>(cloud_.points_.data()), open3d::core::Dtype::Float32, {size(), 3});

//     open3d::t::geometry::PointCloud cloud(positions);
//     cloud.SetPointAttr("intensities", intensities);

//     return cloud;
// }