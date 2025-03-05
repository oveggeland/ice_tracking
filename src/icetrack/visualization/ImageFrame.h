#pragma once

#include <ros/ros.h>

#include <open3d/t/geometry/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "Camera.h"

class ImageFrame{
public:
    ImageFrame(const double ts, const cv::Mat& img, const open3d::t::geometry::PointCloud& pcd, const Camera& camera);

    // Resize image and pixel coordinates with some scaling factor 's'
    void scale(double s); 

    // Accessors
    int pointCount() const { return elevation_.size(); }

    // Return image with imposed pointcloud
    double getStamp() const { return ts_; }
    const cv::Mat& getRawImage() const { return img_; }

    // Impose image
    cv::Mat getImposedImage(const Eigen::VectorXf& values, int color_map, float min = NAN, float max = NAN) const;
    cv::Mat getImposedImage(const cv::Mat& colors) const;                       // From color map
    cv::Mat getImposedImage(const cv::Scalar c = cv::Scalar(0, 255, 0)) const;  // From constant color

    cv::Mat getImposedElevationImage() const { return getImposedImage(elevation_, cv::COLORMAP_JET, -1.0f, 3.0f); };
    cv::Mat getImposedIntensityImage() const { return getImposedImage(intensity_, cv::COLORMAP_JET, 0.0f, 50.0f); };
    cv::Mat getImposedDeformationImage() const { return getImposedImage(deformation_, cv::COLORMAP_MAGMA, 0.0f, 0.2f); };
    cv::Mat getImposedTimeDeltaImage() const { return getImposedImage(dt_, cv::COLORMAP_SPRING); };

    // Accessors
    const Eigen::VectorXf& elevation() const { return elevation_; }
    const Eigen::VectorXf& intensity() const { return intensity_; }
    const Eigen::VectorXf& deformation() const { return deformation_; }
    const Eigen::VectorXf& dt() const { return dt_; }

    const Eigen::Matrix2Xf& uv() const { return uv_; }
    const std::vector<int>& inliers() const { return inliers_; }
private:
    // Image    
    double ts_;
    cv::Mat img_;

    // Cloud
    Eigen::VectorXf elevation_;
    Eigen::VectorXf intensity_;
    Eigen::VectorXf deformation_;
    Eigen::VectorXf dt_;

    // Projection
    Eigen::Matrix2Xf uv_;
    std::vector<int> inliers_;
};