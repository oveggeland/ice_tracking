#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include "frontend/LidarFrontEnd.h"
#include "frontend/FrameBuffer.h"

#include "visualization/Camera.h"

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
    
    void visualize(cv::Mat& img, const Eigen::Matrix3Xf& points, const Eigen::Matrix4f& cTw) {
        if (points.cols() >= 0){
            // Project all points in one step
            Eigen::Matrix2Xf uv = camera_.project(points, cTw);  // Efficient batch operation

            // Draw only the valid points
            for (int i = 0; i < uv.cols(); ++i) {
                if (camera_.inBounds(uv.col(i))) {
                    cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, cv::Scalar(0, 255, 0), -1);
                }
            }
        }

        display("Projected", img);
    }

    void generateDataset(double t_img, cv::Mat& img, const Eigen::Matrix3Xf& r_world, const Eigen::Matrix4f& cTw) {
        int min_points = 1000;
        if (r_world.cols() < min_points){
            ROS_WARN("Not enough points to generate dataset");
            return;
        }

        Eigen::Matrix3Xf r_cam = (cTw.block<3, 3>(0, 0) * r_world).colwise() + cTw.block<3, 1>(0, 3);
        Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam);
        

        for (int i = 0; i < uv.cols(); i++){
            if (camera_.inBounds(uv.col(i))){
                // Push to file
            }
        }
    }

    /*
    Get pose that projects points from nav frame to camera frame.
    */
    bool getInverseCameraPose(double ts, Eigen::Matrix4f& cTw){
        gtsam::Pose3 wTb;
        if (!pose_graph_.poseQuery(ts, wTb)){
            ROS_WARN_STREAM("Pose query failed at: " << std::fixed << ts);
            return false;
        }

        cTw = wTb.compose(bTc_).inverse().matrix().cast<float>();
        return true;
    }

    /*
    Main entry point to processing routines.
    */
    void processImage(double ts, cv::Mat& img){
        // Get pose
        Eigen::Matrix4f cTw;
        if (!getInverseCameraPose(ts, cTw))
            return;

        // Get points
        const Eigen::Matrix3Xf r_world = frame_buffer_.getPointsWithin(ts - offset_, ts + offset_);

        if (generate_dataset_)
            generateDataset(ts, img, r_world, cTw);

        if (display_)
            visualize(img, r_world, cTw);
    }

    /*
    Check image buffer to see if it is processing time
    */
    void checkBuffer(){
        for (auto it = image_buffer_.begin(); it != image_buffer_.end(); ){
            double t_img = it->first;
            if (ros::Time::now().toSec() - t_img > delay_){
                processImage(t_img, it->second);
                it = image_buffer_.erase(it);
            }
            break;
        }
    }

    /*
    New image available
    */
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        double ts = msg->header.stamp.toSec();
        cv::Mat img = camera_.getUndistortedImage(msg);

        image_buffer_[ts] = img;
        checkBuffer();
    }

    /*
    Display image (or update display if window already exists)
    */
    void display(const std::string& name, const cv::Mat& img){
        cv::imshow(name, img);
        cv::waitKey(1);
    }

private:
    Camera camera_;
    const FrameBuffer& frame_buffer_;
    const PoseGraph& pose_graph_;

    gtsam::Pose3 bTc_;

    std::map<double, cv::Mat> image_buffer_;

    // Configuration
    bool generate_dataset_ = false;
    bool display_ = true;
    double delay_ = 1.0; // Wait this time to process image
    double offset_ = 2.0;   // Use points from +- 2 seconds of the image acquisition time
};
