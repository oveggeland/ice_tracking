#include "visualization/Camera.h"

Camera::Camera(const std::string& intrinsics_file) {
    loadIntrinsicsFromFile(intrinsics_file);
}

cv::Mat Camera::getUndistortedImage(const sensor_msgs::Image::ConstPtr& msg) const {
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics_.f_x, 0, intrinsics_.c_x,
                                            0, intrinsics_.f_y, intrinsics_.c_y,
                                            0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << intrinsics_.distortion_coeffs[0], intrinsics_.distortion_coeffs[1],
                                                    intrinsics_.distortion_coeffs[2], intrinsics_.distortion_coeffs[3]);
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, K, distCoeffs);
    return undistorted_image;
}

cv::Mat Camera::getDistortedImage(const sensor_msgs::Image::ConstPtr& msg) const {
    return cv_bridge::toCvCopy(msg, "bgr8")->image;
}

Eigen::Matrix2Xf Camera::projectPoints(const Eigen::Matrix3Xf& points, const Eigen::Matrix4f& cTw, bool distort) const {
    Eigen::Matrix3f K = intrinsics_.getProjectionMatrix();
    Eigen::Matrix<float, 3, 4> image_transform = K * cTw.topRows(3);
    Eigen::Matrix3Xf proj_points = (image_transform.leftCols(3) * points).colwise() + image_transform.col(3);

    Eigen::Matrix2Xf uv = (proj_points.topRows(2).array().rowwise() / proj_points.row(2).array()).matrix();
    if (distort)
        undistortPoints(uv);
    return uv;
}

Eigen::Matrix2Xf Camera::projectPoints(const Eigen::Matrix3Xf& points, bool distort) const {
    Eigen::Matrix3Xf proj_points = intrinsics_.getProjectionMatrix() * points;
    Eigen::Matrix2Xf uv = (proj_points.topRows(2).array().rowwise() / proj_points.row(2).array()).matrix();
    if (distort)
        undistortPoints(uv);
    return uv;
}

void Camera::undistortPoints(Eigen::Matrix2Xf& uv) const {
    Eigen::Matrix2Xf uv_distorted = uv;
    for (int i = 0; i < uv.cols(); ++i) {
        double x = (uv(0, i) - intrinsics_.c_x) / intrinsics_.f_x;
        double y = (uv(1, i) - intrinsics_.c_y) / intrinsics_.f_y;
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double r6 = r2 * r4;
        double x_dist = x * (1 + intrinsics_.distortion_coeffs[0] * r2 + intrinsics_.distortion_coeffs[1] * r4)
                     + 2 * intrinsics_.distortion_coeffs[2] * x * y + intrinsics_.distortion_coeffs[3] * (r2 + 2 * x * x);
        double y_dist = y * (1 + intrinsics_.distortion_coeffs[0] * r2 + intrinsics_.distortion_coeffs[1] * r4)
                     + 2 * intrinsics_.distortion_coeffs[3] * x * y + intrinsics_.distortion_coeffs[2] * (r2 + 2 * y * y);
        uv_distorted(0, i) = intrinsics_.f_x * x_dist + intrinsics_.c_x;
        uv_distorted(1, i) = intrinsics_.f_y * y_dist + intrinsics_.c_y;
    }
    uv = uv_distorted;
}

void Camera::loadIntrinsicsFromFile(const std::string& intrinsics_file) {
    try {
        YAML::Node config = YAML::LoadFile(intrinsics_file);
        if (config["camera_model"].as<std::string>() != "pinhole") {
            ROS_ERROR("Only pinhole camera model is supported.");
            return;
        }
        intrinsics_.distortion_model = config["distortion_model"].as<std::string>();
        if (intrinsics_.distortion_model != "radtan") {
            ROS_ERROR("Only 'radtan' distortion model is supported.");
            return;
        }
        intrinsics_.distortion_coeffs = config["distortion_coeffs"].as<std::vector<double>>();
        std::vector<double> intrinsics = config["intrinsics"].as<std::vector<double>>();
        if (intrinsics.size() == 4) {
            intrinsics_.f_x = intrinsics[0];
            intrinsics_.f_y = intrinsics[1];
            intrinsics_.c_x = intrinsics[2];
            intrinsics_.c_y = intrinsics[3];
        }
        std::vector<int> resolution = config["resolution"].as<std::vector<int>>();
        if (resolution.size() == 2) {
            intrinsics_.w = resolution[0];
            intrinsics_.h = resolution[1];
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load camera intrinsics from file: %s", e.what());
    }
}
