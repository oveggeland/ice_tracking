#include "visualization/Camera.h"

Camera::Camera(const ros::NodeHandle& nh){
    // Load calibration
    std::string extrinsics_file = getParamOrThrow<std::string>(nh, "/ext_file");
    cTb_ = cTb(extrinsics_file);

    std::string intrinsics_file = getParamOrThrow<std::string>(nh, "/int_file");
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


Eigen::Matrix2Xf Camera::projectFromWorld(const Eigen::Matrix3Xf& r_world, bool undistort) const {
    Eigen::Matrix<float, 3, 4> image_transform = intrinsics_.getProjectionMatrix() * cTw_.topRows<3>(); // Find world -> image transformation

    Eigen::Matrix3Xf uv_h = (image_transform.leftCols(3) * r_world).colwise() + image_transform.col(3); // Get homogenous image coordinates
    Eigen::Matrix2Xf uv = (uv_h.topRows(2).array().rowwise() / uv_h.row(2).array()).matrix();           // Normalize to get pixel coordinates

    if (undistort)
        undistortPoints(uv);

    return uv;
}


Eigen::Matrix2Xf Camera::projectFromCam(const Eigen::Matrix3Xf& r_cam, bool undistort) const {
    Eigen::Matrix3Xf uv_h = intrinsics_.getProjectionMatrix() * r_cam;                          // Get homogenous image coordinates
    Eigen::Matrix2Xf uv = (uv_h.topRows(2).array().rowwise() / uv_h.row(2).array()).matrix();   // Normalize to get pixel coordinates

    if (undistort)
        undistortPoints(uv);

    return uv;
}


std::vector<int> Camera::getInliers(const Eigen::Matrix2Xf& uv) const{
    const int num_points = uv.cols();

    std::vector<int> inliers;
    inliers.reserve(num_points);

    for (int i = 0; i < num_points; ++i){
        if (inBounds(uv.col(i)))
            inliers.push_back(i);
    }
    return inliers;
}

std::vector<bool> Camera::getInlierMask(const Eigen::Matrix2Xf& uv) const{
    const int num_points = uv.cols();

    std::vector<bool> mask;
    mask.reserve(num_points);

    for (int i = 0; i < num_points; ++i){
        mask.push_back(inBounds(uv.col(i)));
    }
    return mask;
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
