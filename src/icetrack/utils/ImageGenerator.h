#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include "frontend/LidarFrontEnd.h"
#include "frontend/FrameBuffer.h"
#include "utils/Camera.h"
#include "utils/ros_params.h"

class ImageGenerator {
public:
    ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end) 
        : frame_buffer_(lidar_front_end.frameBuffer()), 
          camera_(getParamOrThrow<std::string>(nh, "int_file")) {
        // TODO: Setup any additional processing
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        cv::Mat img = camera_.getUndistortedImage(msg);

        ROS_INFO_STREAM("Image callback");

        // Preview image (optional)
        cv::imshow("Undistorted Image", img);
        cv::waitKey(1);
    }

private:
    Camera camera_;
    const FrameBuffer& frame_buffer_;
};
