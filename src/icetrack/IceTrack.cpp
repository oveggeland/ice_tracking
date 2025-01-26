#include "icetrack/IceTrack.h"

IceTrack::IceTrack(ros::NodeHandle nh) 
    : sensors_(nh), pose_estimator_(nh), cloud_manager_(nh, sensors_) {
    
    // Diagnostics file and object
    std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
    std::string diag_file = joinPath(outpath, "diag/diag.csv");
    makePath(diag_file);

    diag_ = Diagnostics(diag_file);
}

void IceTrack::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    pose_estimator_.imuCallback(msg);

    diag_.diagEnd();
}

void IceTrack::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    pose_estimator_.gnssCallback(msg);

    diag_.diagEnd();
}

void IceTrack::pclSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();

    diag_.diagStart(ts);

    pose_estimator_.lidarCallback(msg);

    diag_.diagEnd();
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
        t_head_ = t_msg; // TODO: Maybe this should be done in callback?
    }
}