#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "frontend/FrameBuffer.h"

/*
Abstraction to publish cloud messages based on the content of FrameBuffer
*/
class CloudPublisher{
public:
    // Constructor
    CloudPublisher(ros::NodeHandle& nh, const FrameBuffer& frame_buffer);

    // Interface to publish cloud
    void publishCloud();

private:
    // Frame buffer holds cloud frames, which are assembled to construct pointclouds
    const FrameBuffer& frame_buffer_;

    // Publishing
    ros::Publisher cloud_pub_;                              // Publishing object
    sensor_msgs::PointCloud2 cloud_msg_;                    // Message to publish   

    void initializePublisher(ros::NodeHandle& nh);          // Initialize cloud_pub_
    void initializeMessage(ros::NodeHandle& nh);            // Initialize cloud_msg_

    void fillMessage();                                     // Fill cloud_msg_
};

// Defines the memory map of a point in the cloud message
#pragma pack(push, 1)
struct PackedPointXYZIT {
    Eigen::Vector3f pos;
    uint8_t i;
    double t;
};
#pragma pack(pop)