#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <gtsam/base/Vector.h>

class Imu{
    public:
        // Constructor
        Imu();
        Imu(ros::NodeHandle nh);

        // Interface
        void newMessage(const sensor_msgs::Imu::ConstPtr& msg);
        
        const double& getTimeStamp() const;
        const gtsam::Vector3& getAcc() const;
        const gtsam::Vector3& getRate() const;

    private:    
        double ts_;
        gtsam::Vector3 acc_;
        gtsam::Vector3 rate_;

        void setAcc(const sensor_msgs::Imu::ConstPtr& msg);
        void setRate(const sensor_msgs::Imu::ConstPtr& msg);
};