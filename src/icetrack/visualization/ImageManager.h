#pragma once

#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "backend/PoseGraph.h"
#include "frontend/CloudManager.h"

#include "ImageFrame.h"
#include "ImageFrameOutput.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/file_system.h"

class ImageManager {
public:
    ImageManager(ros::NodeHandle& nh, const PoseGraph& pose_graph, const CloudManager& cloud_manager);

    // Interface
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
    // Required module references
    const PoseGraph& pose_graph_;
    const CloudManager& cloud_manager_;
    
    // Camera object (dealing with intrisics, extrinsics, projection, etc.)
    Camera camera_;
    ImageFrameOutput output_;

    // ROS
    ros::NodeHandle nh_;
    std::deque<ros::Timer> timer_buffer_;

    // Main entry point for images
    void processImage(double t_img, const cv::Mat& img);
    void display(const std::string& window_name, const cv::Mat& img) const;
    
    // Configuration
    bool enabled_;              // Enable image generation
    bool display_;              // Display image
    double delay_;       // Delay before image is processed (should be at least max_offset_)
    double offset_;             // Maximum offset between image and lidar points to be considered
};