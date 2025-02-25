#include "ImageOutput.h"

ImageOutput::ImageOutput(ros::NodeHandle& nh, const std::string& folder, const std::string& topic, int queue_size) {
    if (!folder.empty()) {
        std::filesystem::create_directories(folder);
        image_folder_ = folder;
    }

    if (!topic.empty()) {
        image_pub_ = nh.advertise<sensor_msgs::Image>(topic, queue_size);
    }
}
    

void ImageOutput::newImage(double ts, const cv::Mat& img) const{
    if (!image_folder_.empty()) {
        saveImage(ts, img);
    }

    if (image_pub_) {  // Check if publisher is initialized
        publishImage(ts, img);
    }
}
    

void ImageOutput::saveImage(double ts, const cv::Mat& img) const {
    std::stringstream fname;
    fname << std::fixed << static_cast<int>(1000 * ts) << ".png";
    std::string fpath = image_folder_ + "/" + fname.str();

    if (!cv::imwrite(fpath, img)) {
        ROS_WARN("Failed to save image at %s", fpath.c_str());
    }
}
    

void ImageOutput::publishImage(double ts, const cv::Mat& img) const {
    if (image_pub_.getNumSubscribers() == 0) {
        return;
    }

    std::string encoding = (img.channels() == 1) ? "mono8" : "bgr8";
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, img).toImageMsg();
    msg->header.stamp = ros::Time(ts);
    image_pub_.publish(msg);
}