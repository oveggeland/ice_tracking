#include "FixedLagMapper.h"

FixedLagMapper::FixedLagMapper(ros::NodeHandle& nh) 
    : pose_graph_(nh), 
    cloud_manager_(nh, pose_graph_),
    image_manager_(nh, pose_graph_, cloud_manager_) {
    
    // Setup callback sequencer
    sequencer_ = CallbackSequencer(getParamOrThrow<double>(nh, "/navigation/safe_delay"));
    
    // Setup subscribers
    setupSubscribers(nh);
}


void FixedLagMapper::setupSubscribers(ros::NodeHandle& nh){
    std::string imu_topic = getParamOrThrow<std::string>(nh, "/imu_topic");
    std::string gnss_topic = getParamOrThrow<std::string>(nh, "/gnss_topic");
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");
    std::string image_topic = getParamOrThrow<std::string>(nh, "/image_topic");

    int imu_queue_size = getParamOrThrow<int>(nh, "/imu_queue_size");
    int gnss_queue_size = getParamOrThrow<int>(nh, "/gnss_queue_size");
    int lidar_queue_size = getParamOrThrow<int>(nh, "/lidar_queue_size");
    int image_queue_size = getParamOrThrow<int>(nh, "/image_queue_size");

    imu_sub_ = nh.subscribe(imu_topic, imu_queue_size, &FixedLagMapper::imuCallback, this);
    gnss_sub_ = nh.subscribe(gnss_topic, gnss_queue_size, &FixedLagMapper::gnssCallback, this);
    lidar_sub_ = nh.subscribe(lidar_topic, lidar_queue_size, &FixedLagMapper::lidarCallback, this);
    image_sub_ = nh.subscribe(image_topic, image_queue_size, &FixedLagMapper::imageCallback, this);
}


// Subscriber callbacks. All we do is add the callbacks to the sequencer to assert chronological order of messages
void FixedLagMapper::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapper::imuSafeCallback, this, msg));
}

void FixedLagMapper::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapper::gnssSafeCallback, this, msg));
}

void FixedLagMapper::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapper::lidarSafeCallback, this, msg));
}

void FixedLagMapper::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&FixedLagMapper::imageSafeCallback, this, msg));
}

// Sequenced "safe" callbacks. These will always be called in chronological order according to message timestamps. 
void FixedLagMapper::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    pose_graph_.imuCallback(msg);
}

void FixedLagMapper::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    pose_graph_.gnssCallback(msg);
}

void FixedLagMapper::lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_manager_.lidarCallback(msg);
}

void FixedLagMapper::imageSafeCallback(const sensor_msgs::Image::ConstPtr& msg) {
    image_manager_.imageCallback(msg);
}