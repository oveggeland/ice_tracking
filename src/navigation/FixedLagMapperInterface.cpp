#include "FixedLagMapperInterface.h"

FixedLagMapperInterface::FixedLagMapperInterface(ros::NodeHandle& nh) : fixed_lag_mapper_(nh) {
    // Setup sequencer and subscribers
    sequencer_ = CallbackSequencer(getParamOrThrow<double>(nh, "/navigation/safe_delay"));

    std::string imu_topic = getParamOrThrow<std::string>(nh, "/imu_topic");
    std::string gnss_topic = getParamOrThrow<std::string>(nh, "/gnss_topic");
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");

    int imu_queue_size = getParamOrThrow<int>(nh, "/imu_queue_size");
    int gnss_queue_size = getParamOrThrow<int>(nh, "/gnss_queue_size");
    int lidar_queue_size = getParamOrThrow<int>(nh, "/lidar_queue_size");

    imu_sub_ = nh.subscribe(imu_topic, imu_queue_size, &FixedLagMapperInterface::imuCallback, this);
    gnss_sub_ = nh.subscribe(gnss_topic, gnss_queue_size, &FixedLagMapperInterface::gnssCallback, this);
    lidar_sub_ = nh.subscribe(lidar_topic, lidar_queue_size, &FixedLagMapperInterface::lidarCallback, this);
}


// Subscriber callbacks. All we do is add the callbacks to the sequencer to assert chronological order of messages
void FixedLagMapperInterface::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapperInterface::imuSafeCallback, this, msg));
}
void FixedLagMapperInterface::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapperInterface::gnssSafeCallback, this, msg));
}
void FixedLagMapperInterface::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapperInterface::lidarSafeCallback, this, msg));
}


void FixedLagMapperInterface::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    fixed_lag_mapper_.imuCallback(msg);
}
void FixedLagMapperInterface::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    fixed_lag_mapper_.gnssCallback(msg);
}
void FixedLagMapperInterface::lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    fixed_lag_mapper_.lidarCallback(msg);
}