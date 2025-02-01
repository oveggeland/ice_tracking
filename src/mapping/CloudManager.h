#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <open3d/t/geometry/PointCloud.h>

#include "mapping/CloudGenerator.h"
#include "mapping/CloudProcessor.h"
#include "mapping/CloudAnalyzer.h"

#include "utils/ros_params.h"
#include "utils/CallbackSequencer.h"

class CloudManager2{
public:
    CloudManager2(ros::NodeHandle nh);

private:
    // Callback sequencing
    CallbackSequencer sequencer_;
    
    // Lidar subscriber
    ros::Subscriber lidar_sub_;
    int lidar_seq_ = 0;
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Pose subscriber
    ros::Subscriber pose_sub_;
    int pose_seq_ = 0;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseSafeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Window management
    double t0_window_ = 0.0;
    double window_size_;
    void checkWindowTime(double ts);
    void analyzeWindow(double ts);

    // Submodules for specific cloud tasks
    CloudGenerator cloud_generator_;
    CloudProcessor cloud_processor_;
    CloudAnalyzer cloud_analyzer_;
};