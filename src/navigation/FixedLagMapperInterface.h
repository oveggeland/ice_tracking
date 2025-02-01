#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "utils/CallbackSequencer.h"
#include "navigation/FixedLagMapper.h"


class FixedLagMapperInterface{
public:
    FixedLagMapperInterface(ros::NodeHandle& nh);

private:
    FixedLagMapper fixed_lag_mapper_; 

    // Subscribers and subscriber callbacks
    ros::Subscriber imu_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber lidar_sub_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Safe callback sequencing
    CallbackSequencer sequencer_;
    
    void imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};