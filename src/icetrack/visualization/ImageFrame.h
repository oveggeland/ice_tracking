#pragma once

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <algorithm>


class ImageFrame{
public:
    ImageFrame() : scale_(1.0) {};
    ImageFrame(const double scale) : scale_(scale) {};

    void reset(const size_t capacity){
        elevation_.resize(0);
        intensity_.resize(0);
        uv_.resize(0);

        elevation_.reserve(capacity);
        intensity_.reserve(capacity);
        uv_.reserve(capacity);
    }

    // Setters
    void setImage(const cv::Mat& img) {
        cv::resize(img, img_, cv::Size(), scale_, scale_);
    };

    void addPoint(const float elevation, const float intensity, const Eigen::Vector2f& uv){
        elevation_.push_back(elevation);
        intensity_.push_back(intensity);
        uv_.push_back(uv*scale_);
    };
    // Functionality to generate useful projections
    int size() const { return uv_.size(); }
    const cv::Mat& getRawImage() const { return img_; }
    cv::Mat getImposedImage(const cv::Scalar& c = cv::Scalar(0, 255, 0)) const;
    cv::Mat getImposedElevationImage(int color_map = cv::COLORMAP_JET, float v_min=-1, float v_max=3) const;
    cv::Mat getImposedIntensityImage(int color_map = cv::COLORMAP_COOL, float v_min=0, float v_max=50) const;

private:
    double scale_ = 1.0;
    
    // Image data
    cv::Mat img_;

    // Cloud data
    std::vector<float> elevation_;
    std::vector<float> intensity_;
    std::vector<Eigen::Vector2f> uv_;

    // Helpers
    void drawPoint(const cv::Mat& img, const cv::Scalar& c) const;
    cv::Mat getImposedImage(const cv::Mat& colors) const;
    cv::Mat getImposedImage(const std::vector<float>& values, int color_map, float v_min, float v_max) const;
};