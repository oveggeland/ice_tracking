#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>


#include "LidarFrontEnd.h"
#include "PoseGraph.h"

#include "PoseGraphManager.h"
#include "CloudManager.h"

#include "utils/CallbackSequencer.h"

class FixedLagMapper{
public:
    FixedLagMapper(ros::NodeHandle& nh);

private:
    // Main modules
    PoseGraphManager pose_graph_manager_;
    CloudManager cloud_manager_;

    // Refactored modules
    LidarFrontEnd lidar_front_end_;
    PoseGraph pose_graph_;

    // Subscribers and subscriber callbacks
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);


    // Callback sequencing
    CallbackSequencer sequencer_;
    
    void imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};