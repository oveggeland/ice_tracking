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
    double fx, fy;    // Focal Length
    double cx, cy;    // Optical center
    int w, h;           // Image size

    double k1, k2, p1, p2; // Radtan

    Eigen::Matrix3f getProjectionMatrix() const {
        return (Eigen::Matrix3f() << fx, 0, cx, 0, fy, cy, 0, 0, 1).finished();
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

    std::vector<int> getInliers(const Eigen::Matrix2Xf& uv) const;
    std::vector<bool> getInlierMask(const Eigen::Matrix2Xf& uv) const;
    inline bool inBounds(const Eigen::Vector2f& uv) const {
        return uv.x() >= 0 && uv.y() >= 0 && uv.x() < intrinsics_.w && uv.y() < intrinsics_.h;
    }

    inline void updateTransform(const gtsam::Pose3& wTb) { cTw_ = cTb_.compose(wTb.inverse()).matrix().cast<float>(); }

    const Eigen::Matrix4f& getPose() const { return cTw_; }
    const CameraIntrinsics& intrinsics() const { return intrinsics_; }

private:
    // Calibration
    gtsam::Pose3 cTb_;
    CameraIntrinsics intrinsics_;
    void loadIntrinsicsFromFile(const std::string& intrinsics_file);

    // Current transformation
    Eigen::Matrix4f cTw_;
};
