#include "pointcloud.h"

void visualizeCloud(const PointCloud& cloud) {
    visualizeCloud(std::make_shared<PointCloud>(cloud));
}
void visualizeCloud(const PointCloudPtr cloud){
    open3d::visualization::DrawGeometries({cloud});
}
void visualizeCloud(const TensorCloud& cloud){
    visualizeCloud(cloud.ToLegacy());
}
void visualizeCloud(const TensorCloudPtr cloud){
    visualizeCloud(cloud->ToLegacy());
}

void visualizeAlignment(const PointCloudPtr pcd0, const PointCloudPtr pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align){
    auto pcd1_initial = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_initial.matrix()));
    auto pcd1_aligned = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_align.matrix()));

    pcd0->PaintUniformColor({1, 0, 0});
    pcd1->PaintUniformColor({0, 0, 1});
    pcd1_initial->PaintUniformColor({0, 0, 1});
    pcd1_aligned->PaintUniformColor({0, 0, 1});

    open3d::visualization::DrawGeometries({pcd0, pcd1_initial}, "Initial alignment");
    open3d::visualization::DrawGeometries({pcd0, pcd1_aligned}, "ICP aligned");
}
void visualizeAlignment(const PointCloud pcd0, const PointCloud pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align){
    visualizeAlignment(std::make_shared<PointCloud>(pcd0), std::make_shared<PointCloud>(pcd1), T_initial, T_align);
}
void visualizeAlignment(const TensorCloud& pcd0, const TensorCloud& pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align){
    visualizeAlignment(pcd0.ToLegacy(), pcd1.ToLegacy(), T_initial, T_align);
}


int getCloudSize(const TensorCloud& cloud){
    return cloud.GetPointPositions().GetShape(0);
}
int getCloudSize(const TensorCloudPtr cloud){
    return getCloudSize(*cloud);
}
int getCloudSize(const PointCloud& cloud){
    return cloud.points_.size();
}
int getCloudSize(const PointCloudPtr cloud){
    return cloud->points_.size();
}


// Shared memory
TensorCloud EigenToTensorCloud(Eigen::Matrix3Xf positions) {
    // Create Open3D tensor from the Eigen matrix. Since we are using `Eigen::Map`, there is no memory copy.
    open3d::core::Tensor tensor(positions.data(), open3d::core::Dtype::Float32, {positions.cols(), 3});

    // Create the TensorCloud from the Open3D Tensor
    return TensorCloud(tensor);
}