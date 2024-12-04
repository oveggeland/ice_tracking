#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "icetrack/navigation/icenav.h"
#include "icetrack/mapping/icemap.h"

#include <sys/sysinfo.h>

class IceTrack{
public:
    IceTrack(ros::NodeHandle nh, double lag);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imgCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
    // Optimization
    IceNav nav_;
    IceMap map_;

    // Diag stuff
    std::ofstream f_diag_;

    double diag_t0_wall_;
    double diag_t1_wall_;
    double diag_mem0_;
    double diag_mem1_;
    int diag_msg_type_;
    double diag_msg_stamp_;

    void diagStart(int msg_type, double ts);
    void diagEnd();
    void diagWrite();
};