#pragma once

#include <open3d/Open3D.h>

using PointCloud = open3d::geometry::PointCloud;
using PointCloudPtr = std::shared_ptr<PointCloud>;

using TensorCloud = open3d::t::geometry::PointCloud;
using TensorCloudPtr = std::shared_ptr<TensorCloud>;

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

TensorCloud EigenToTensorCloud(Eigen::Matrix3Xf positions);