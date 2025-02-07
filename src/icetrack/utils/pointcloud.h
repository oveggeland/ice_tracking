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