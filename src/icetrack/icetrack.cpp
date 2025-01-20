#include "icetrack/icetrack.h"

IceTrack::IceTrack(){}

IceTrack::IceTrack(ros::NodeHandle nh): nh_(nh){
    sensors_ = std::make_shared<SensorSystem>(nh_);

    nav_ = IceNav(nh_, sensors_);
    cloud_manager_ = CloudManager(nh_, sensors_);
    
    // Diagnostics file and object
    std::string outpath = getParamOrThrow<std::string>(nh_, "/outpath");
    std::string diag_file = joinPath(outpath, "diag/diag.csv");
    makePath(diag_file);

    diag_ = Diagnostics(joinPath(outpath, "diag/diag.csv"));
}

void IceTrack::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    nav_.imuMeasurement(msg);

    diag_.diagEnd();
}

void IceTrack::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    // Update navigation
    nav_.gnssMeasurement(msg);

    // Provide new pose to cloud manager
    if (nav_.isInit())
        cloud_manager_.newPose(msg->header.stamp.toSec(), nav_.getPose());

    diag_.diagEnd();
}

void IceTrack::pclSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    // Parse incoming lidar points!
    sensors_->lidar()->addFrame(msg);
    
    diag_.diagEnd();
}

void IceTrack::shipSafeCallback(const icetrack::ShipNavigation::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    // Parse incoming lidar points!
    nav_.shipNavigationMeasurement(msg);

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

void IceTrack::shipCallback(const icetrack::ShipNavigation::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::shipSafeCallback, this, msg));
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