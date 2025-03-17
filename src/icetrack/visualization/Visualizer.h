#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "ImageFrame.h"
#include "Camera.h"
#include "frontend/point_types.h"

#include "utils/ros_params.h"
#include "utils/conversions.h"

class Visualizer {
public:
    Visualizer(ros::NodeHandle& nh);

private:    
    // Camera object (dealing with intrisics, extrinsics, projection, etc.)
    Camera camera_;
    ImageFrame frame_;

    // Helper functions for image visualization
    void visualize(double t_img, const cv::Mat& img);
    void display(const std::string& window_name, const cv::Mat& img) const;
    
    // Config
    bool display_;              // Display image
    double delay_;              // Delay visualization
    double scale_;              // Scale down original image resolution for visualization (reduced complexity)

    bool publish_;              // Publish as ros topic
    ros::Publisher publisher_;  // Publisher object
    void publish(const cv::Mat& img);



    // Image callback
    ros::Subscriber image_sub_;
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    std::map<double, cv::Mat> image_buffer_;
    void checkImageBuffer();

    // Cloud callback
    ros::Subscriber cloud_sub_;
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    sensor_msgs::PointCloud2::ConstPtr cloud_msg_;
    void processLatestCloud();

    ///// Pose subscribing and interpolation ///// 
    ros::Subscriber pose_sub_;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    double t0_ = 0.0;
    double t1_ = 0.0;
    gtsam::Pose3 pose0_;
    gtsam::Pose3 pose1_;

    gtsam::Pose3 getPose(const double ts);
};