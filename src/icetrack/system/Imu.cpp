#include "icetrack/system/Imu.h"

// Extract accelerometer vector from Imu message
gtsam::Vector3 getAcc(const sensor_msgs::Imu::ConstPtr& msg){
    return gtsam::Vector3(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
}

// Extract angular velocity vector from Imu message
gtsam::Vector3 getRate(const sensor_msgs::Imu::ConstPtr& msg){
    return gtsam::Vector3(
        msg->angular_velocity.x, 
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
}

Imu::Imu(){
    // Default constructor
}

Imu::Imu(ros::NodeHandle nh){
    getParamOrThrow(nh, "/system/imu/acc_norm_threshold", acc_norm_threshold_);
    getParamOrThrow(nh, "/system/imu/rate_norm_threshold", rate_norm_threshold_);
}

bool Imu::newMessage(const sensor_msgs::Imu::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    if (ts <= meas_.ts){
        ROS_WARN("Imu: Timestamp is prior or equal to previous IMU message...");
        return false;
    }

    gtsam::Vector3 acc = getAcc(msg);
    gtsam::Vector3 rate = getRate(msg);

    if (acc.norm() > acc_norm_threshold_ || rate.norm() > rate_norm_threshold_){
        ROS_WARN("Imu: Measurement norms are higher than the accepted threshold...");
        return false;
    }

    meas_ = ImuMeasurement{
        ts,
        acc,
        rate
    };

    return true;
}

const ImuMeasurement& Imu::getMeasurement() const{
    return meas_;
}