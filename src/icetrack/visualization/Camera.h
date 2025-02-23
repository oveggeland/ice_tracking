#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h> // Transformations

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // If you're using cv_bridge
#include <sensor_msgs/Image.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h> // For YAML loading

#include "utils/ros_params.h"
#include "utils/calibration.h"

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
    Camera(const ros::NodeHandle& nh);

    // Image from ROS message
    cv::Mat getDistortedImage(const sensor_msgs::Image::ConstPtr& msg) const;
    cv::Mat getUndistortedImage(const sensor_msgs::Image::ConstPtr& msg) const;

    // Point projections
    Eigen::Matrix2Xf projectFromCam(const Eigen::Matrix3Xf& r_cam, bool undistort) const; // Cam frame point projection
    Eigen::Matrix2Xf projectFromWorld(const Eigen::Matrix3Xf& points, bool distort) const; // World frame point projection
    
    void undistortPoints(Eigen::Matrix2Xf& uv) const;

    std::vector<bool> getInlierMask(const Eigen::Matrix2Xf& uv) const;
    inline bool inBounds(const Eigen::Vector2f& uv) const {
        return uv.x() >= 0 && uv.y() >= 0 && uv.x() < intrinsics_.w && uv.y() < intrinsics_.h;
    }

    inline void updateTransform(const gtsam::Pose3& wTb) { cTw_ = cTb_.compose(wTb.inverse()).matrix().cast<float>(); }

private:
    // Calibration
    gtsam::Pose3 cTb_;
    CameraIntrinsics intrinsics_;
    void loadIntrinsicsFromFile(const std::string& intrinsics_file);

    // Current transformation
    Eigen::Matrix4f cTw_;
};
