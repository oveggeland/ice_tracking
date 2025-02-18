#pragma once

#include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "backend/PoseGraph.h"

#include "frontend/LidarFrontEnd.h"
#include "frontend/FrameBuffer.h"

#include "visualization/Camera.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/file_system.h"
#include "utils/pointcloud.h"

class ImageGenerator {
public:
    ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end, const PoseGraph& pose_graph);

    // Interface
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
    // Required classes
    const FrameBuffer& frame_buffer_;
    const PoseGraph& pose_graph_;
    
    // ROS
    ros::NodeHandle nh_;
    std::map<double, ros::Timer> timer_buffer_;

    // Helper functions
    void drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers) const;
    bool getInverseCameraPose(double ts, Eigen::Matrix4f& cTw) const;
    std::vector<bool> getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const;

    void processImage(double t_img, cv::Mat& img);
    
    // Camera
    gtsam::Pose3 bTc_;
    Camera camera_;

    // Configuration
    bool enabled_;              // Enable image generation
    bool display_;              // Display image
    double delay_;       // Delay before image is processed (should be at least max_offset_)
    double offset_;             // Maximum offset between image and lidar points to be considered

    // Publishing
    void publishImage(double t_img, const cv::Mat& img) const;
    ros::Publisher image_pub_;
};