#include "icetrack/icetrack.h"

double getMemoryUsage() {
    struct sysinfo info;
    if (sysinfo(&info) == 0) {
        return info.freeram * info.mem_unit / (1024.0 * 1024.0);
    } else {
        ROS_WARN("Failed to get system info!");
        return 0.0;
    }
}


void IceTrack::diagStart(int msg_type, double ts){
    diag_t0_wall_ = ros::Time::now().toSec();
    diag_mem0_ = getMemoryUsage();
    diag_msg_type_ = msg_type;
    diag_msg_stamp_ = ts;
}

void IceTrack::diagEnd(){
    diag_t1_wall_ = ros::Time::now().toSec();
    diag_mem1_ = getMemoryUsage();
    diagWrite();
}

void IceTrack::diagWrite(){
    f_diag_ << diag_msg_type_ << ",";
    f_diag_ << diag_msg_stamp_ << ",";
    f_diag_ << diag_t0_wall_ << ",";
    f_diag_ << diag_t1_wall_ << ",";
    f_diag_ << diag_mem0_ << ",";
    f_diag_ << diag_mem1_ << std::endl;
}




IceTrack::IceTrack(ros::NodeHandle nh, double lag){
    nav_ = IceNav(nh, lag);
    map_ = IceMap();

    f_diag_ = std::ofstream("/home/oskar/icetrack/output/diag.csv");
    f_diag_ << "msg_type,t_stamp,t0_wall,t1_wall,mem0,mem1" << std::endl << std::fixed;
}

void IceTrack::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    diagStart(0, msg->header.stamp.toSec());

    nav_.imuCallback(msg);

    diagEnd();
}


void IceTrack::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    diagStart(1, msg->header.stamp.toSec());

    // Update navigation
    nav_.gnssCallback(msg);

    // Update pose map with latest pose from nav
    double t_pose;
    gtsam::Pose3 current_pose = nav_.getLastPose(t_pose);

    map_.updatePoseMap(current_pose, t_pose);

    diagEnd();
}


void IceTrack::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    diagStart(2, msg->header.stamp.toSec());

    map_.pclCallback(msg);

    diagEnd();
}


void IceTrack::imgCallback(const sensor_msgs::Image::ConstPtr& msg){
}