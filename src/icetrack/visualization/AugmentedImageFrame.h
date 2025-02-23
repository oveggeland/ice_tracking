#pragma once

#include <ros/ros.h>

#include <open3d/t/geometry/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Camera.h"

class AugmentedImageFrame{
public:
    AugmentedImageFrame(const double ts, const cv::Mat& img, const open3d::t::geometry::PointCloud& pcd, const Camera& camera);

    // Accessors
    int pointCount() const { return elevation_.size(); }
    int inlierCount() const;

    // Return image with imposed pointcloud
    cv::Mat getImposedImage() const;

private:
    // Image    
    double ts_;
    cv::Mat img_;

    // Cloud
    Eigen::VectorXf elevation_;
    Eigen::VectorXf intensity_;
    Eigen::VectorXf deformation_;

    // Projection
    Eigen::Matrix2Xf uv_;
    std::vector<bool> inlier_mask_;
};