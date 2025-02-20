#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <open3d/t/geometry/PointCloud.h>

#include "utils/ros_params.h"

/*
Abstraction to publish cloud messages.
*/
class CloudPublisher{
public:
    // Constructor
    CloudPublisher(ros::NodeHandle& nh);

    // Interface to publish cloud
    void publishRawCloud(const open3d::t::geometry::PointCloud& cloud);
    void publishProcessedCloud(const open3d::t::geometry::PointCloud& cloud);

private:
    // Raw cloud
    ros::Publisher raw_cloud_pub_;
    sensor_msgs::PointCloud2 raw_cloud_msg_;  

    void setupRawCloudPublisher(ros::NodeHandle& nh);
    void fillRawCloudMessage(const open3d::t::geometry::PointCloud& cloud); 

    // Processed cloud
    ros::Publisher processed_cloud_pub_;                
    sensor_msgs::PointCloud2 processed_cloud_msg_; 

    void setupProcessedCloudPublisher(ros::NodeHandle& nh);
    void fillProcessedCloudMessage(const open3d::t::geometry::PointCloud& cloud);
};

// Defines the memory map of a point in the cloud message
#pragma pack(push, 1)
struct PackedPointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

struct PackedPointXYZDI {
    float x;
    float y;
    float z;
    float deformation;
    float intensity;
};
#pragma pack(pop)