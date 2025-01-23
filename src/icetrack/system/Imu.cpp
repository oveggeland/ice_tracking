#include "icetrack/system/Imu.h"

Imu::Imu(){
    // Default constructor
}

Imu::Imu(ros::NodeHandle nh){
    // Potential for including configuration here
}

void Imu::newMessage(const sensor_msgs::Imu::ConstPtr& msg){
    ts_ = msg->header.stamp.toSec();
    setAcc(msg);
    setRate(msg);
}

void Imu::setAcc(const sensor_msgs::Imu::ConstPtr& msg){
    acc_ = gtsam::Vector3(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
}

void Imu::setRate(const sensor_msgs::Imu::ConstPtr& msg){
    rate_ = gtsam::Vector3(
        msg->angular_velocity.x, 
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
}

const double& Imu::getTimeStamp() const{
    return ts_;
}

const gtsam::Vector3& Imu::getAcc() const{
    return acc_;
}

const gtsam::Vector3& Imu::getRate() const{
    return rate_;
}