#pragma once

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
    Camera(CameraIntrinsics intrinsics) : intrinsics_(intrinsics) {}
    Camera(const std::string& intrinsics_file) {
        loadIntrinsicsFromFile(intrinsics_file);
    }

    // Undistort the image
    cv::Mat getUndistortedImage(const sensor_msgs::Image::ConstPtr& msg) const {
        // Convert ROS Image message to OpenCV Mat
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Set up OpenCV undistortion matrices
        cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics_.f_x, 0, intrinsics_.c_x,
                                                0, intrinsics_.f_y, intrinsics_.c_y,
                                                0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << intrinsics_.distortion_coeffs[0], intrinsics_.distortion_coeffs[1],
                                                        intrinsics_.distortion_coeffs[2], intrinsics_.distortion_coeffs[3]);

        cv::Mat undistorted_image;
        cv::undistort(image, undistorted_image, K, distCoeffs);

        // Return the undistorted image
        return undistorted_image;
    }

    // Distorted image (just returns the original image for now)
    cv::Mat getDistortedImage(const sensor_msgs::Image::ConstPtr& msg) const {
        return cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    // Project 3D points onto the image plane
    Eigen::Matrix2Xf project(const Eigen::Matrix3Xf& points, const Eigen::Matrix4f& cam_pose) const {
        // Combine intrinsics and camera pose to create the projection matrix
        Eigen::Matrix3f K = intrinsics_.getProjectionMatrix();
        Eigen::Matrix<float, 3, 4> image_transform = K*cam_pose.topRows(3);

        // Apply the camera transformation (camera pose and intrinsic matrix)
        Eigen::Matrix3Xf proj_points = (image_transform.leftCols(3) * points).colwise() + image_transform.col(3);

        // Normalize to get pixel coordinates (image plane)
        return (proj_points.topRows(2).array().rowwise() / proj_points.row(2).array()).matrix();
    }

    inline bool inBounds(const Eigen::Vector2f& uv) const{
        return uv.x() >= 0 && uv.y() >= 0 && uv.x() < intrinsics_.w && uv.y() < intrinsics_.h;
    }

private:
    CameraIntrinsics intrinsics_;

    // Function to load intrinsics from a YAML file
    void loadIntrinsicsFromFile(const std::string& intrinsics_file) {
        try {
            YAML::Node config = YAML::LoadFile(intrinsics_file);

            // Parse camera model parameters
            if (config["camera_model"].as<std::string>() != "pinhole") {
                ROS_ERROR("Only pinhole camera model is supported.");
                return;
            }

            // Parse distortion model
            intrinsics_.distortion_model = config["distortion_model"].as<std::string>();
            if (intrinsics_.distortion_model != "radtan") {
                ROS_ERROR("Only 'radtan' distortion model is supported.");
                return;
            }

            // Parse distortion coefficients
            intrinsics_.distortion_coeffs = config["distortion_coeffs"].as<std::vector<double>>();

            // Parse intrinsic parameters
            std::vector<double> intrinsics = config["intrinsics"].as<std::vector<double>>();
            if (intrinsics.size() == 4) {
                intrinsics_.f_x = intrinsics[0];
                intrinsics_.f_y = intrinsics[1];
                intrinsics_.c_x = intrinsics[2];
                intrinsics_.c_y = intrinsics[3];
            }

            // Parse resolution
            std::vector<int> resolution = config["resolution"].as<std::vector<int>>();
            if (resolution.size() == 2) {
                intrinsics_.w = resolution[0];
                intrinsics_.h = resolution[1];
            }

        } catch (const YAML::Exception& e) {
            ROS_ERROR("Failed to load camera intrinsics from file: %s", e.what());
        }
    }
};
