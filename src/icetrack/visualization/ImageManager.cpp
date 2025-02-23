#include "visualization/ImageManager.h"

ImageManager::ImageManager(ros::NodeHandle& nh, const PoseGraph& pose_graph, const CloudManager& cloud_manager)
    : nh_(nh), camera_(nh), pose_graph_(pose_graph), cloud_manager_(cloud_manager){
    // Load config
    getParamOrThrow<bool>(nh, "/image_generator/enabled", enabled_);
    getParamOrThrow<bool>(nh, "/image_generator/display", display_);
    getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
    getParamOrThrow<double>(nh, "/image_generator/offset", offset_);
}

// void ImageManager::drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers) const{
//     cv::Scalar color(0, 255, 0);

//     for (int i = 0; i < uv.cols(); ++i) {
//         if (inliers[i])
//             cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, color, -1);
//     }
// }

// bool ImageManager::getInverseCameraPose(double ts, Eigen::Matrix4f& cTw) const{
//     gtsam::Pose3 wTb;
//     if (!pose_graph_.poseQuery(ts, wTb)) {
//         ROS_WARN_STREAM("Pose query failed at: " << std::fixed << ts);
//         return false;
//     }
//     cTw = wTb.compose(bTc_).inverse().matrix().cast<float>();
//     return true;
// }

// std::vector<bool> ImageManager::getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const {
//     std::vector<bool> inliers;
//     inliers.reserve(uv.cols());

//     for (int i = 0; i < uv.cols(); ++i) {
//         if (camera_.inBounds(uv.col(i)) && r_cam(2, i) > 0)
//             inliers.push_back(true);
//         else
//             inliers.push_back(false);
//     }
//     return inliers;
// }

void ImageManager::display(const std::string& window_name, const cv::Mat& img) const {
    cv::imshow(window_name, img);

    // Wait for a key indefinitely if 'p' (ASCII 112) is pressed
    char key = cv::waitKey(1);  // Non-blocking wait
    if (key == 'p' || key == ' ') {  // Pause on 'p' or spacebar
        while (cv::waitKey(0) < 0);  // Wait indefinitely until any key is pressed
    }
    else if (key == 'q')
        ros::shutdown();
}


// // Crop to [0, 255] and apply colormap. Clip values to min and max if provided.
// cv::Mat getColorMap(const Eigen::VectorXf& values, int color_map, float min = NAN, float max = NAN) {
//     // Use provided min/max if valid, otherwise compute from values
//     float min_val = std::isnan(min) ? values.minCoeff() : min;
//     float max_val = std::isnan(max) ? values.maxCoeff() : max;

//     // Prevent division by zero (if all values are the same)
//     if (max_val == min_val) {
//         max_val += 1.0f;
//     }

//     // Normalize values to [0, 1] within the given range
//     Eigen::VectorXf normalized = ((values.array() - min_val) / (max_val - min_val)).cwiseMax(0.0f).cwiseMin(1.0f);

//     // Convert to OpenCV grayscale format [0, 255]
//     cv::Mat grayscale(1, values.size(), CV_8UC1);
//     for (int i = 0; i < values.size(); ++i) {
//         grayscale.at<float>(0, i) = static_cast<float>(normalized(i) * 255.0f);
//     }

//     // Apply colormap
//     cv::Mat colored;
//     cv::applyColorMap(grayscale, colored, color_map);
    
//     return colored;
// }

// cv::Mat drawProjection(const Eigen::Matrix2Xf uv, const std::vector<bool> inliers, const cv::Mat& colors, int w, int h){
//     cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
//     for (int i = 0; i < uv.cols(); ++i) {
//         if (inliers[i]) {
//             cv::Vec3b color = colors.at<cv::Vec3b>(0, i); // Extract color from colormap
//             cv::Scalar c(color[0], color[1], color[2]);

//             // Draw circle using the extracted color
//             cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 5, c, -1);
//         }
//     }
//     return img;
// }

void ImageManager::processImage(double t_img, cv::Mat& img){
    ROS_INFO_STREAM("Process image with timestamp: " << std::fixed << t_img);
    timer_buffer_.pop_front();

    // Query pose (world to camera transformation)
    gtsam::Pose3 wTb;
    if (!pose_graph_.poseQuery(t_img, wTb)){
        ROS_ERROR_STREAM("ImageManager::processImage - Could not get pose at " << std::fixed << t_img);
        return;
    }
    camera_.updateTransform(wTb);

    // Query cloud (in world frame)
    const auto cloud = cloud_manager_.cloudQuery(true, t_img - offset_, t_img + offset_);
    int num_points = cloud.IsEmpty()? 0: cloud.GetPointPositions().GetShape(0);
    if (num_points == 0){
        ROS_WARN_STREAM("ImageManager::processImage - No cloud points available at: " << std::fixed << t_img);
    }

    // Generate augmented image
    AugmentedImageFrame augmented(t_img, img, cloud, camera_);
    cv::Mat imposed = augmented.getImposedImage();

    display("Imposed", imposed);
}

// Main entry point. Unpack the image and schedule processing. 
void ImageManager::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!enabled_)
        return;

    // Get timestamp and image
    double ts = msg->header.stamp.toSec();
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Schedule processing
    timer_buffer_.push_back(nh_.createTimer(
        ros::Duration(delay_), // Duration
        boost::bind(&ImageManager::processImage, this, ts, img), // Callback
        true // One-shot
    ));
}

// void ImageManager::publishImage(double t_img, const cv::Mat& img) const{
//     if (!image_pub_.getNumSubscribers())
//         return;

//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//     msg->header.stamp = ros::Time(t_img);
//     image_pub_.publish(msg);
// }