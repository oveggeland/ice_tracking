#pragma once

#include <open3d/Open3D.h>

using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<PointCloud>;

using TensorCloud = open3d::t::geometry::PointCloud;
using TensorCloudPtr = std::shared_ptr<TensorCloud>;

void compareClouds(const PointCloud& cloud0, const PointCloud& cloud1);

void visualizeCloud(const PointCloud& cloud);
void visualizeCloud(const PointCloudPtr cloud);

void visualizeCloud(const TensorCloud& cloud);
void visualizeCloud(const TensorCloudPtr cloud);

void visualizeAlignment(const PointCloudPtr pcd0, const PointCloudPtr pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align);
void visualizeAlignment(const PointCloud pcd0, const PointCloud pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align);
void visualizeAlignment(const TensorCloud& pcd0, const TensorCloud& pcd1, Eigen::Matrix4d T_initial, Eigen::Matrix4d T_align);

int getCloudSize(const PointCloud& cloud);
int getCloudSize(const PointCloudPtr cloud);
int getCloudSize(const TensorCloud& cloud);
int getCloudSize(const TensorCloudPtr cloud);

// To legacy cloud, deep copy
PointCloudPtr EigenToPointCloudPtr(const Eigen::Matrix3Xf& positions);


const open3d::core::Tensor EigenToTensorFloat(const Eigen::MatrixXf& mat);

// To tensor cloud, shallow copy
TensorCloud EigenToTensorCloud(Eigen::Matrix3Xf& positions);