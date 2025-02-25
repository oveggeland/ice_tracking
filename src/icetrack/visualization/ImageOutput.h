#pragma once

#include <filesystem>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  // If you're using cv_bridge

class ImageOutput {
public:
    ImageOutput(){}
    ImageOutput(ros::NodeHandle& nh, const std::string& folder = "", const std::string& topic = "", int queue_size = 10);

    void newImage(double ts, const cv::Mat& img) const;

private:
    std::string image_folder_;
    ros::Publisher image_pub_;

    void saveImage(double ts, const cv::Mat& img) const;
    void publishImage(double ts, const cv::Mat& img) const;
};