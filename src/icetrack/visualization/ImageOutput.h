#pragma once

#include <filesystem>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // If you're using cv_bridge

#include "utils/file_system.h"

class ImageOutput {
public:
    ImageOutput(){}
    ImageOutput(ros::NodeHandle& nh, const std::string& folder = "", const std::string& topic = "", const int queue_size = 10);

    void newImage(const std::string& label, const double ts, const cv::Mat& img) const;

private:
    std::string image_folder_;
    ros::Publisher image_pub_;

    void saveImage(const std::string& label, const cv::Mat& img) const;
    void publishImage(const double ts, const cv::Mat& img) const;
};