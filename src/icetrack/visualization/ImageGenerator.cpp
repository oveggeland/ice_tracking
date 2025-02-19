#include "visualization/ImageGenerator.h"

ImageGenerator::ImageGenerator(ros::NodeHandle& nh, const CloudManager& cloud_manager, const PoseGraph& pose_graph)
    : nh_(nh), frame_buffer_(cloud_manager.frameBuffer()), pose_graph_(pose_graph),
      camera_(getParamOrThrow<std::string>(nh, "int_file")) {
    // Load calibration
    bTc_ = bTc(getParamOrThrow<std::string>(nh, "ext_file"));
    
    // Load parameters
    getParamOrThrow<bool>(nh, "/image_generator/enabled", enabled_);
    getParamOrThrow<bool>(nh, "/image_generator/display", display_);
    getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
    getParamOrThrow<double>(nh, "/image_generator/offset", offset_);

    // Initialize publisher
    std::string topic = getParamOrThrow<std::string>(nh, "/image_generator/topic");
    image_pub_ = nh.advertise<sensor_msgs::Image>(topic, 10);

    // Launch image view with image rempaped as topic
    if (display_){
        std::string cmd = "rosrun image_view image_view image:=" + topic + " &";
        int ret = system(cmd.c_str());
        if (ret != 0) {
            ROS_WARN_STREAM("Failed to launch image_view with command: " << cmd);
        }
    }
}

void ImageGenerator::drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers) const{
    cv::Scalar color(0, 255, 0);

    for (int i = 0; i < uv.cols(); ++i) {
        if (inliers[i])
            cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, color, -1);
    }
}

bool ImageGenerator::getInverseCameraPose(double ts, Eigen::Matrix4f& cTw) const{
    gtsam::Pose3 wTb;
    if (!pose_graph_.poseQuery(ts, wTb)) {
        ROS_WARN_STREAM("Pose query failed at: " << std::fixed << ts);
        return false;
    }
    cTw = wTb.compose(bTc_).inverse().matrix().cast<float>();
    return true;
}

std::vector<bool> ImageGenerator::getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const {
    std::vector<bool> inliers;
    inliers.resize(uv.cols());

    for (int i = 0; i < uv.cols(); ++i) {
        if (camera_.inBounds(uv.col(i)) && r_cam(2, i) > 0)
            inliers[i] = true;
        else
            inliers[i] = false;
    }
    return inliers;
}

// Crop to [0, 255] and apply colormap. Clip values to min and max if provided.
cv::Mat getColorMap(const Eigen::VectorXf& values, int color_map, float min = NAN, float max = NAN) {
    // Use provided min/max if valid, otherwise compute from values
    float min_val = std::isnan(min) ? values.minCoeff() : min;
    float max_val = std::isnan(max) ? values.maxCoeff() : max;

    // Prevent division by zero (if all values are the same)
    if (max_val == min_val) {
        max_val += 1.0f;
    }

    // Normalize values to [0, 1] within the given range
    Eigen::VectorXf normalized = ((values.array() - min_val) / (max_val - min_val)).cwiseMax(0.0f).cwiseMin(1.0f);

    // Convert to OpenCV grayscale format [0, 255]
    cv::Mat grayscale(1, values.size(), CV_8UC1);
    for (int i = 0; i < values.size(); ++i) {
        grayscale.at<uint8_t>(0, i) = static_cast<uint8_t>(normalized(i) * 255.0f);
    }

    // Apply colormap
    cv::Mat colored;
    cv::applyColorMap(grayscale, colored, color_map);
    
    return colored;
}

cv::Mat drawProjection(const Eigen::Matrix2Xf uv, const std::vector<bool> inliers, const cv::Mat& colors, int w, int h){
    cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
    for (int i = 0; i < uv.cols(); ++i) {
        if (inliers[i]) {
            cv::Vec3b color = colors.at<cv::Vec3b>(0, i); // Extract color from colormap
            cv::Scalar c(color[0], color[1], color[2]);

            // Draw circle using the extracted color
            cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 5, c, -1);
        }
    }
    return img;
}

void ImageGenerator::processImage(double t_img, cv::Mat& img){
    // Pop timer
    timer_buffer_.erase(t_img);

    // Query pose
    Eigen::Matrix4f cTw;
    if (!getInverseCameraPose(t_img, cTw))
        return;

    // Query points
    double t0 = t_img - offset_;
    double t1 = t_img + offset_;
    CloudFrame::Ptr frame = frame_buffer_.getPoints(t0, t1, false, true, true, true);

    if (frame->empty())
        return;
        
    // Project points
    const Eigen::Matrix3Xf r_world = frame->global();
    const Eigen::Matrix3Xf r_cam = (cTw.topLeftCorner<3, 3>() * r_world).colwise() + cTw.topRightCorner<3, 1>();
    const Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam, false);

    // Find inliers
    const std::vector<bool> inliers = getInliers(r_cam, uv);
    
    // Make elevation image
    Eigen::VectorXf elevation = -r_world.row(2);
    auto elev_colors = getColorMap(elevation, cv::COLORMAP_JET, -1.0, 3.0);
    auto img_elev = drawProjection(uv, inliers, elev_colors, img.cols, img.rows);

    cv::Mat elev_blend;
    cv::addWeighted(img, 1, img_elev, 0.5, 0, elev_blend);

    // Make intensity image
    Eigen::VectorXf intensities = frame->intensities().cast<float>();
    auto intensity_colors = getColorMap(intensities, cv::COLORMAP_COOL, 0, 50);
    auto img_intensity = drawProjection(uv, inliers, intensity_colors, img.cols, img.rows);

    cv::Mat intensity_blend;
    cv::addWeighted(img, 1, img_intensity, 0.5, 0, intensity_blend);

    // Stack images (img, img_elev, img_intensity)
    cv::Mat stacked_img;
    cv::hconcat(img, elev_blend, stacked_img);
    cv::hconcat(stacked_img, intensity_blend, stacked_img);

    // Resize image
    cv::resize(stacked_img, stacked_img, cv::Size(), 0.5, 0.5);

    // Publish images
    publishImage(t_img, stacked_img);
}

// Main entry point. Unpack the image and schedule processing. 
void ImageGenerator::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!enabled_)
        return;

    // Get timestamp and image
    double ts = msg->header.stamp.toSec();
    cv::Mat img = camera_.getDistortedImage(msg);

    // Schedule processing
    timer_buffer_[ts] = nh_.createTimer(
        ros::Duration(delay_), // Duration
        boost::bind(&ImageGenerator::processImage, this, ts, img), // Callback
        true // One-shot
    );
}

void ImageGenerator::publishImage(double t_img, const cv::Mat& img) const{
    if (!image_pub_.getNumSubscribers())
        return;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    msg->header.stamp = ros::Time(t_img);
    image_pub_.publish(msg);
}