#include "visualization/Camera.h"

Camera::Camera(const ros::NodeHandle& nh){
    // Load calibration
    std::string extrinsics_file = getParamOrThrow<std::string>(nh, "/ext_file");
    cTb_ = cTb(extrinsics_file);

    std::string intrinsics_file = getParamOrThrow<std::string>(nh, "/int_file");
    loadIntrinsicsFromFile(intrinsics_file);
}

Eigen::Vector2f Camera::projectPoint(const Eigen::Vector3f& r_cam, bool undistort) const {
    // Normalize to image plane
   Eigen::Vector2f xy_img = r_cam.head<2>() / r_cam.z();

    // Undistortion
    if (undistort)
        xy_img = undistortPoint(xy_img);

    // Pixel projection
    return {
        xy_img.x() * intrinsics_.fx + intrinsics_.cx,
        xy_img.y() * intrinsics_.fy + intrinsics_.cy
    };
}


Eigen::Vector2f Camera::undistortPoint(const Eigen::Vector2f& xy) const {
    const float x = xy.x();
    const float y = xy.y();

    // Precompute radial factor
    const float r2 = x * x + y * y;
    const float r4 = r2 * r2;
    const float radial_factor = (1 + intrinsics_.k1 * r2 + intrinsics_.k2 * r4);

    // Compute undistorted coordinates
    return {
        x * radial_factor + 2 * intrinsics_.p1 * x * y + intrinsics_.p2 * (r2 + 2 * x * x),
        y * radial_factor + 2 * intrinsics_.p2 * x * y + intrinsics_.p1 * (r2 + 2 * y * y)
    };
}


void Camera::loadIntrinsicsFromFile(const std::string& intrinsics_file) {
    try {
        YAML::Node config = YAML::LoadFile(intrinsics_file);

        // Load distortion
        std::vector<double> dist_coeffs = config["distortion_coeffs"].as<std::vector<double>>();
        intrinsics_.k1 = dist_coeffs[0];
        intrinsics_.k2 = dist_coeffs[1];
        intrinsics_.p1 = dist_coeffs[2];
        intrinsics_.p2 = dist_coeffs[3];
        
        // Load projection matrix
        std::vector<double> intrinsics = config["intrinsics"].as<std::vector<double>>();
        intrinsics_.fx = intrinsics[0];
        intrinsics_.fy = intrinsics[1];
        intrinsics_.cx = intrinsics[2];
        intrinsics_.cy = intrinsics[3];
        
        std::vector<int> resolution = config["resolution"].as<std::vector<int>>();
        intrinsics_.w = resolution[0];
        intrinsics_.h = resolution[1];
        
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load camera intrinsics from file: %s", e.what());
    }
}
