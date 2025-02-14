#pragma once

#include <fstream>
#include <iomanip>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "frontend/LidarFrontEnd.h"
#include "frontend/FrameBuffer.h"

#include "visualization/Camera.h"
#include "visualization/SensorFrame.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/file_system.h"

class ImageGenerator {
public:
    ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end, const PoseGraph& pose_graph) 
        :   frame_buffer_(lidar_front_end.frameBuffer()), 
            pose_graph_(pose_graph),
            camera_(getParamOrThrow<std::string>(nh, "int_file")) {
        // TODO: Setup any additional processing
        bTc_ = bTc(getParamOrThrow<std::string>(nh, "ext_file"));

        getParamOrThrow<bool>(nh, "/image_generator/generate_dataset", generate_dataset_);
        getParamOrThrow<bool>(nh, "/image_generator/display", display_);
        getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
        getParamOrThrow<double>(nh, "/image_generator/offset", offset_);

        // Output stuff
        std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
        image_path_ = joinPath(outpath, "dataset/images/");
        bin_path_ = joinPath(outpath, "dataset/projections/");
        makePath(image_path_, true);
        makePath(bin_path_, true);
    }
    
    void drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv) {
        for (int i = 0; i < uv.cols(); ++i) {
            if (camera_.inBounds(uv.col(i)))
                cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, cv::Scalar(0, 255, 0), -1);
        }
    }
    
    void generateDataset(double t_img, cv::Mat& img, const Eigen::Matrix3Xf& r_world, const Eigen::Matrix4f& cTw) {
        int min_points = 1000;
        if (r_world.cols() < min_points) {
            ROS_WARN("Not enough points to generate dataset");
            return;
        }

        // Project points from world to camera frame
        Eigen::Matrix3Xf r_cam = (cTw.block<3, 3>(0, 0) * r_world).colwise() + cTw.block<3, 1>(0, 3);
        Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam);

        // Generate timestamp-based filename
        std::ostringstream timestamp_stream;
        timestamp_stream << std::fixed << std::setprecision(6) << t_img;  // For 6 decimal places
        std::string timestamp_str = timestamp_stream.str();

        std::string binary_file_path = joinPath(bin_path_, timestamp_str + ".bin");
        std::string image_file_path = joinPath(image_path_, timestamp_str + ".png");

        // Open the binary file for writing
        std::ofstream binary_file(binary_file_path, std::ios::binary);
        if (!binary_file.is_open()) {
            ROS_ERROR("Failed to open binary file for writing: %s", binary_file_path.c_str());
            return;
        }

        // Write the r_world, r_cam, and uv matrices into the binary file
        int num_points = r_world.cols();
        binary_file.write(reinterpret_cast<const char*>(&num_points), sizeof(int));

        // Write r_world (3xN matrix)
        for (int i = 0; i < 3; ++i) {
            binary_file.write(reinterpret_cast<const char*>(r_world.row(i).data()), num_points * sizeof(float));
        }

        // Write r_cam (3xN matrix)
        for (int i = 0; i < 3; ++i) {
            binary_file.write(reinterpret_cast<const char*>(r_cam.row(i).data()), num_points * sizeof(float));
        }

        // Write uv (2xN matrix)
        for (int i = 0; i < 2; ++i) {
            binary_file.write(reinterpret_cast<const char*>(uv.row(i).data()), num_points * sizeof(float));
        }

        // Close the binary file
        binary_file.close();

        // Now, save the image as a PNG
        if (!cv::imwrite(image_file_path, img)) {
            ROS_ERROR("Failed to save image to %s", image_file_path.c_str());
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

    std::vector<int> getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const{
        std::vector<int> inliers;
        inliers.reserve(uv.cols());

        for (int i = 0; i < uv.cols(); ++i){
            if (camera_.inBounds(uv.col(i)) && r_cam(2, i) > 0)
                inliers.push_back(i);
        }

        return inliers;
    }

    /*
    Main entry point to processing routines.
    */
    void processImage(double t_img, cv::Mat& img){
        // Get pose
        Eigen::Matrix4f cTw;
        if (!getInverseCameraPose(t_img, cTw))
            return;

        // Get points
        const Eigen::Matrix3Xf r_world = frame_buffer_.getPointsWithin(t_img - offset_, t_img + offset_);
        const Eigen::Matrix3Xf r_cam = (cTw.block<3, 3>(0, 0) * r_world).colwise() + cTw.block<3, 1>(0, 3);
        const Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam);

        drawPoints(img, uv);
        if (display_)
            display("Projected", img);
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
    bool generate_dataset_;
    bool display_;
    double delay_; // Wait this time to process image
    double offset_;   // Use points from +- 2 seconds of the image acquisition time

    std::string image_path_;
    std::string bin_path_;
};
