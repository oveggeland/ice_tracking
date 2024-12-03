#include "icetrack/icetrack.h"


IceTrack::IceTrack(ros::NodeHandle nh, double lag){
    nav_ = IceNav(nh, lag);
    map_ = IceMap();
}

void IceTrack::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Update navigation
    nav_.imuCallback(msg);
}


void IceTrack::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    // Update navigation
    nav_.gnssCallback(msg);

    // Update pose map with latest pose from nav
    double t_pose;
    gtsam::Pose3 current_pose = nav_.getLastPose(t_pose);

    map_.updatePoseMap(current_pose, t_pose);
}


void IceTrack::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    map_.pclCallback(msg);
}


void IceTrack::imgCallback(const sensor_msgs::Image::ConstPtr& msg){
}