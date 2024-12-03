#include "icetrack/icetrack.h"


IceTrack::IceTrack(ros::NodeHandle nh, double lag){
    nav_ = IceNav(nh, lag);
    map_ = IceMap();
}

void IceTrack::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    nav_.imuCallback(msg);
}


void IceTrack::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    nav_.gnssCallback(msg);
}


void IceTrack::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    nav_.pclCallback(msg);
}


void IceTrack::imgCallback(const sensor_msgs::Image::ConstPtr& msg){
}