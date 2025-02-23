#pragma once

#include <ros/ros.h>

#include <open3d/t/geometry/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Camera.h"

class AugmentedImageFrame{
public:
    AugmentedImageFrame(const double ts, const cv::Mat& img, const open3d::t::geometry::PointCloud& pcd, const Camera& camera, const double scale);

    // Accessors
    int pointCount() const { return elevation_.size(); }

    // Return image with imposed pointcloud
    cv::Mat getImage() const { return img_; }


    // Impose image
    cv::Mat getImposedImage(const Eigen::VectorXf& values, int color_map, float min = NAN, float max = NAN) const;
    cv::Mat getImposedImage(const cv::Mat& colors) const;                       // From color map
    cv::Mat getImposedImage(const cv::Scalar c = cv::Scalar(0, 255, 0)) const;  // From constant color

    cv::Mat getImposedElevationImage() const { return getImposedImage(elevation_, cv::COLORMAP_JET, -1.0f, 3.0f); };
    cv::Mat getImposedIntensityImage() const { return getImposedImage(intensity_, cv::COLORMAP_JET, 0.0f, 50.0f); };
    cv::Mat getImposedDeformationImage() const { return getImposedImage(deformation_, cv::COLORMAP_MAGMA, 0.0f, 0.2f); };

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
    std::vector<int> inliers_;
};