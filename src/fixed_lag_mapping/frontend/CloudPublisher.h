#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>

#include "point_types.h"

#include "utils/ros_params.h"

/*
Abstraction to publish cloud messages.
*/
class CloudPublisher{
public:
    // Constructor
    CloudPublisher(ros::NodeHandle& nh);

    // Interface to publish cloud
    void publishFrame(const std::vector<Eigen::Vector3f>& positions, const std::vector<float>& intensities);
    void publishCloud(const std::vector<PointXYZI>& points);

private:
    // Single frame
    ros::Publisher frame_pub_;
    sensor_msgs::PointCloud2 frame_msg_; 

    // Global cloud
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_msg_;  

    // Setup
    void setupPublishers(ros::NodeHandle& nh);
    void initializeMessages();

    // Fill message
    void fillCloudMessage(sensor_msgs::PointCloud2& msg, const std::vector<Eigen::Vector3f>& positions, const std::vector<float>& intensities); 
    void fillCloudMessage(sensor_msgs::PointCloud2& msg, const std::vector<PointXYZI>& points); 
};