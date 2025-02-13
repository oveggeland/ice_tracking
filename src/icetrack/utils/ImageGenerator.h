#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include "frontend/LidarFrontEnd.h"
#include "frontend/FrameBuffer.h"
#include "utils/Camera.h"
#include "utils/ros_params.h"
#include "utils/calibration.h"

class ImageGenerator {
public:
    ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end, const PoseGraph& pose_graph) 
        :   frame_buffer_(lidar_front_end.frameBuffer()), 
            pose_graph_(pose_graph),
            camera_(getParamOrThrow<std::string>(nh, "int_file")) {
        // TODO: Setup any additional processing
        bTc_ = bTc(getParamOrThrow<std::string>(nh, "ext_file"));
    }

    void checkBuffer(){
        double t0 = frame_buffer_.getFirstTimeStamp();
        double t1 = frame_buffer_.getLastTimeStamp();

        for (auto it = image_buffer_.begin(); it != image_buffer_.end(); ){
            double t_img = it->first;
            if (t1 - t_img > 2.0){
                processImage(t_img, it->second);
                it = image_buffer_.erase(it);
            }  
            else{
                return;
            }
        }
    }


    void projectPoints(cv::Mat& img, const Eigen::Matrix3Xf& points, const Eigen::Matrix4f& cTw) {
        if (points.cols() == 0) return;  // No points to process

        // Project all points in one step
        Eigen::Matrix2Xf uv = camera_.project(points, cTw);  // Efficient batch operation

        // Draw only the valid points
        for (int i = 0; i < uv.cols(); ++i) {
            if (camera_.inBounds(uv.col(i))) {
                cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, cv::Scalar(0, 255, 0), -1);
            }
        }

        showImage(img);
    }


    void processImage(double ts, cv::Mat& img){
        gtsam::Pose3 wTb; // Body pose
        if (!pose_graph_.poseQuery(ts, wTb))
            return;
        const Eigen::Matrix4f cTw = wTb.compose(bTc_).inverse().matrix().cast<float>();

        const Eigen::Matrix3Xf points = frame_buffer_.getPointsWithin(ts - 1.0, ts + 1.0);
        if (points.cols() > 0){
            projectPoints(img, points, cTw);
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        double ts = msg->header.stamp.toSec();
        cv::Mat img = camera_.getUndistortedImage(msg);

        image_buffer_[ts] = img;

        checkBuffer();
    }

    void showImage(const cv::Mat& img){
        cv::imshow("Image", img);
        cv::waitKey(1);
    }

private:
    Camera camera_;
    const FrameBuffer& frame_buffer_;
    const PoseGraph& pose_graph_;

    gtsam::Pose3 bTc_;

    std::map<double, cv::Mat> image_buffer_;
};
