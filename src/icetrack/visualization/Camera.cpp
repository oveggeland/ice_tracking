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
    cv::Mat K = (cv::Mat_<double>(3, 3) << intrinsics_.fx, 0, intrinsics_.cx,
                                            0, intrinsics_.fy, intrinsics_.cy,
                                            0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(4, 1) << intrinsics_.k1, intrinsics_.k2,
                                                    intrinsics_.p1, intrinsics_.p2); // Radtan (k1, k2, r1, r2)
    cv::Mat undistorted_image;
    cv::undistort(image, undistorted_image, K, distCoeffs);
    return undistorted_image;
}

cv::Mat Camera::getDistortedImage(const sensor_msgs::Image::ConstPtr& msg) const {
    return cv_bridge::toCvCopy(msg, "bgr8")->image;
}


Eigen::Matrix2Xf Camera::projectFromWorld(const Eigen::Matrix3Xf& r_world, bool undistort) const {
    Eigen::Matrix3Xf r_cam = (cTw_.topLeftCorner<3, 3>() * r_world).colwise() + cTw_.topRightCorner<3, 1>();
    return projectFromCam(r_cam, undistort);
}


Eigen::Matrix2Xf Camera::projectFromCam(const Eigen::Matrix3Xf& r_cam, bool undistort) const {
    // Normalize to image plane
    Eigen::Matrix2Xf xy_img = (r_cam.topRows<2>().array().rowwise() / r_cam.row(2).array()).matrix();

    // Undistort if desired
    if (undistort)
        undistortPoints(xy_img);

    // Pixel projection
    Eigen::Matrix2Xf uv = (xy_img.array().colwise() * Eigen::Array2f(intrinsics_.fx, intrinsics_.fy)).colwise() + Eigen::Array2f(intrinsics_.cx, intrinsics_.cy);
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

void Camera::undistortPoints(Eigen::Matrix2Xf& xy) const {
    const int n_points = xy.cols();
    for (int i = 0; i < n_points; ++i) {
        // Get point reference
        float& x = xy(0, i);
        float& y = xy(1, i);

        // Precompute radial factor
        const float r2 = x * x + y * y;
        const float r4 = r2 * r2;
        const float radial_factor = (1 + intrinsics_.k1 * r2 + intrinsics_.k2 * r4);

        // Compute undistorted coordinates
        const float x_dist = x * radial_factor + 2 * intrinsics_.p1 * x * y + intrinsics_.p2 * (r2 + 2 * x * x);
        const float y_dist = y * radial_factor + 2 * intrinsics_.p2 * x * y + intrinsics_.p1 * (r2 + 2 * y * y);

        // Write back to xy
        x = x_dist;
        y = y_dist;
    }
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
