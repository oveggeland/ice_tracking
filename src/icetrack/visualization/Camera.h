#pragma once

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // If you're using cv_bridge
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h> // For YAML loading

struct CameraIntrinsics {
    double f_x, f_y;    // Focal Length
    double c_x, c_y;    // Optical center
    int w, h;           // Image size

    std::vector<double> distortion_coeffs; // Distortion coefficients (k1, k2, p1, p2)
    std::string distortion_model;          // Distortion model (e.g., "radtan")

    Eigen::Matrix3f getProjectionMatrix() const {
        return (Eigen::Matrix3f() << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1).finished();
    }
};

class Camera {
public:
    Camera(const std::string& intrinsics_file);

    cv::Mat getUndistortedImage(const sensor_msgs::Image::ConstPtr& msg) const;
    cv::Mat getDistortedImage(const sensor_msgs::Image::ConstPtr& msg) const;

    Eigen::Matrix2Xf projectPoints(const Eigen::Matrix3Xf& points, const Eigen::Matrix4f& cTw, bool distort) const;
    Eigen::Matrix2Xf projectPoints(const Eigen::Matrix3Xf& points, bool distort) const;
    void undistortPoints(Eigen::Matrix2Xf& uv) const;

    inline bool inBounds(const Eigen::Vector2f& uv) const {
        return uv.x() >= 0 && uv.y() >= 0 && uv.x() < intrinsics_.w && uv.y() < intrinsics_.h;
    }

private:
    CameraIntrinsics intrinsics_;
    void loadIntrinsicsFromFile(const std::string& intrinsics_file);
};
