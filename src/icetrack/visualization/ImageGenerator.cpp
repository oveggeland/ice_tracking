#include "visualization/ImageGenerator.h"

ImageGenerator::ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end, const PoseGraph& pose_graph)
    : nh_(nh), frame_buffer_(lidar_front_end.frameBuffer()), pose_graph_(pose_graph),
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

    // Project points
    const Eigen::Matrix3Xf r_world = frame->global();
    const Eigen::Matrix3Xf r_cam = (cTw.topLeftCorner<3, 3>() * r_world).colwise() + cTw.topRightCorner<3, 1>();
    const Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam, false);

    // Find inliers
    const std::vector<bool> inliers = getInliers(r_cam, uv);

    // Draw points
    drawPoints(img, uv, inliers);
    publishImage(t_img, img);
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