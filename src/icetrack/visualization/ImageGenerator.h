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
    void drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers) const;


    bool getInverseCameraPose(double ts, Eigen::Matrix4f& cTw) const;
    std::vector<bool> getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const;

    void processImage(double t_img, cv::Mat& img) const;
    void checkBuffer();
    void display(const std::string& name, const cv::Mat& img) const;

    Camera camera_;
    const FrameBuffer& frame_buffer_;
    const PoseGraph& pose_graph_;
    gtsam::Pose3 bTc_;
    std::map<double, cv::Mat> image_buffer_;
    bool generate_dataset_;
    bool display_;
    double delay_;
    double offset_;
    std::string image_path_;
    std::string bin_path_;
};