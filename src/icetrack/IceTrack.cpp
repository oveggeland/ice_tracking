#include "icetrack/IceTrack.h"

IceTrack::IceTrack(ros::NodeHandle nh) 
    : pose_estimator_(nh), cloud_manager_(nh) {
    
    // Initialize message sequencer
    double safe_delay = getParamOrThrow<double>(nh, "/safe_delay");
    sequencer_ = CallbackSequencer(safe_delay);

    // Diagnostics file and object
    std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
    std::string diag_file = joinPath(outpath, "diag/diag.csv");
    makePath(diag_file);

    diag_ = Diagnostics(diag_file);
}

void IceTrack::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    bool pose_update = pose_estimator_.imuCallback(msg);
    if (pose_update)
        cloud_manager_.newPose(pose_estimator_.getTimeStamp(), pose_estimator_.getPose());

    diag_.diagEnd();
}

void IceTrack::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    diag_.diagStart(msg->header.stamp.toSec());

    bool pose_update = pose_estimator_.gnssCallback(msg);
    if (pose_update)
        cloud_manager_.newPose(pose_estimator_.getTimeStamp(), pose_estimator_.getPose());

    diag_.diagEnd();
}

void IceTrack::pclSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();

    diag_.diagStart(ts);

    pose_estimator_.lidarCallback(msg);
    cloud_manager_.lidarCallback(msg);

    diag_.diagEnd();
}

void IceTrack::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::imuSafeCallback, this, msg));
}

void IceTrack::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::gnssSafeCallback, this, msg));
}

void IceTrack::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&IceTrack::pclSafeCallback, this, msg));
}