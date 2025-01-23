#include "icetrack/system/Lidar.h"

Lidar::Lidar(ros::NodeHandle nh){
    getParamOrThrow(nh, "/system/lidar/point_interval", point_interval_);
    point_buffer_ = StampedRingBuffer<RawLidarPoint>(5.0 / point_interval_); // Keep track of at least 5 seconds worth of points

    double min_dist = getParamOrThrow<double>(nh, "/system/lidar/min_dist");
    min_dist_squared_ = min_dist*min_dist;

    double max_dist = getParamOrThrow<double>(nh, "/system/lidar/max_dist");
    max_dist_squared_ = max_dist*max_dist;

    getParamOrThrow(nh, "/system/lidar/min_intensity", min_intensity_);
}


bool Lidar::newMessage(sensor_msgs::PointCloud2::ConstPtr msg){
    double ts_point = msg->header.stamp.toSec() - point_interval_; // Track stamp of points as we iterate
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejections
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
        )
            continue;

        // Range filtering
        double dist_squared = it[0]*it[0] + it[1]*it[1] + it[2]*it[2];
        if (dist_squared < min_dist_squared_ || dist_squared > max_dist_squared_)
            continue;

        // Point accepted
        point_buffer_.addPoint({
            ts_point,
            it[0],
            it[1],
            it[2],
            it[3]
        });        
    }
    
    if (ts_point > ts_head_){
        ts_head_ = ts_point;
        return true;
    }
    return false; // Nothin happened
}