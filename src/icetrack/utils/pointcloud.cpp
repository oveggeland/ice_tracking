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