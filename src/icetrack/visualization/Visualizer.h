#pragma once

#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <open3d/geometry/PointCloud.h>

#include "backend/PoseGraph.h"
#include "frontend/CloudManager.h"

#include "ImageFrame.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/file_system.h"

class Visualizer {
public:
    Visualizer(ros::NodeHandle& nh, const PoseGraph& pose_graph, const CloudManager& cloud_manager);

    // Interface
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
    // Required module references
    const PoseGraph& pose_graph_;
    const CloudManager& cloud_manager_;
    
    // Camera object (dealing with intrisics, extrinsics, projection, etc.)
    Camera camera_;

    // ROS
    ros::NodeHandle nh_;
    std::deque<ros::Timer> timer_buffer_;

    // Helper functions for image visualization
    void visualize(double t_img, const cv::Mat& img);
    void display(const std::string& window_name, const cv::Mat& img) const;
    
    // Config 
    bool enabled_;              // Enable the visualizer

    bool display_;              // Display image

    bool publish_;              // Publish as ros topic
    ros::Publisher publisher_;  // Publisher object

    double delay_;              // Delay visualization
    double scale_;              // Scale down original image resolution for visualization (reduced complexity)
};