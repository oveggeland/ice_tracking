#include "icetrack/icetrack.h"

// Constructor
IceTrack::IceTrack(){
    lidar_ = std::make_shared<LidarHandle>();

    nav_ = IceNav(lidar_);
    cloud_manager_ = CloudManager(lidar_);
    
    diag_ = Diagnostics("/home/oskar/icetrack/output/diag/diag.csv");
}

void IceTrack::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    nav_.imuMeasurement(msg);
}

void IceTrack::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    nav_.gnssMeasurement(msg);

    if (nav_.isInit())
        cloud_manager_.newPose(msg->header.stamp.toSec(), nav_.getPose());
}

void IceTrack::pclSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    lidar_->addFrame(msg);
    
    if (!lidar_->isInit())
        lidar_->init(msg->header.stamp.toSec()); // TODO: Look at how lidar is initialized
}

void IceTrack::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::imuSafeCallback, this, msg));
}

void IceTrack::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::gnssSafeCallback, this, msg));
}

void IceTrack::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::pclSafeCallback, this, msg));
}

void IceTrack::addCallback(double ts, std::function<void()> cb){
    if (ts < t_head_) // Is message even valid?
        return;
    
    callback_buffer_[ts] = cb;

    // Update safe time and check for valid callbacks
    if (ts - safe_delay_ > t_safe_){
        t_safe_ = ts - safe_delay_;
        checkCallbackBuffer();
    }
}

void IceTrack::checkCallbackBuffer(){
    for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
        double t_msg = it->first;

        if (t_msg > t_safe_)
            break;
        
        it->second(); // Callback 

        it = callback_buffer_.erase(it);
        t_head_ = t_msg; // Maybe this should be done in callback?
    }
}