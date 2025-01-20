#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <gtsam/base/Vector.h>

#include "icetrack/common.h"

gtsam::Vector3 getAcc(const sensor_msgs::Imu::ConstPtr& msg);
gtsam::Vector3 getRate(const sensor_msgs::Imu::ConstPtr& msg);

struct ImuMeasurement{
    double ts=0.0;
    gtsam::Vector3 acc;
    gtsam::Vector3 rate;
};

class Imu{
    public:
        // Constructor
        Imu();
        Imu(ros::NodeHandle nh);

        // Interface
        bool newMessage(const sensor_msgs::Imu::ConstPtr& msg);
        const ImuMeasurement& getMeasurement() const;

    private:    
        ImuMeasurement meas_;

        double acc_norm_threshold_;
        double rate_norm_threshold_;
};