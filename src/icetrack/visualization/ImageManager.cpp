#include "visualization/ImageManager.h"

ImageManager::ImageManager(ros::NodeHandle& nh, const PoseGraph& pose_graph, const CloudManager& cloud_manager)
    : nh_(nh), camera_(nh), output_(nh), pose_graph_(pose_graph), cloud_manager_(cloud_manager){

    // Load config
    getParamOrThrow<bool>(nh, "/image_generator/enabled", enabled_);
    getParamOrThrow<bool>(nh, "/image_generator/display", display_);
    getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
    getParamOrThrow<double>(nh, "/image_generator/offset", offset_);
}


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


void ImageManager::processImage(double t_img, const cv::Mat& img){
    timer_buffer_.pop_front();

    // Query pose (world to camera transformation)
    gtsam::Pose3 wTb;
    if (!pose_graph_.poseQuery(t_img, wTb)){
        ROS_ERROR_STREAM("ImageManager::processImage - Could not get pose at " << std::fixed << t_img);
        return;
    }
    camera_.updateTransform(wTb);

    // Query cloud (in world frame)
    const auto cloud = cloud_manager_.cloudQuery(false, t_img - offset_, t_img + offset_);
    int num_points = cloud.IsEmpty()? 0: cloud.GetPointPositions().GetShape(0);
    if (num_points == 0){
        ROS_WARN_STREAM("ImageManager::processImage - No cloud points available at: " << std::fixed << t_img);
    }

    // Generate augmented image
    ImageFrame img_frame(t_img, img, cloud, camera_);
    output_.newImageFrame(img_frame);
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