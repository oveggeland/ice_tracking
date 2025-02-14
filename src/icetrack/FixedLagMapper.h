#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "frontend/LidarFrontEnd.h"
#include "backend/PoseGraph.h"
#include "visualization/ImageGenerator.h"

#include "utils/CallbackSequencer.h"

class FixedLagMapper{
public:
    FixedLagMapper(ros::NodeHandle& nh);

private:
    // Main modules
    PoseGraph pose_graph_;
    LidarFrontEnd lidar_front_end_;
    ImageGenerator image_generator_;

    // Subscribers and subscriber callbacks
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber image_sub_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    // Callback sequencing
    CallbackSequencer sequencer_;
    
    void imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imageSafeCallback(const sensor_msgs::Image::ConstPtr& msg);
};