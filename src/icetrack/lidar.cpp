#include "icetrack/lidar.h"

Lidar::Lidar(){}

Lidar::Lidar(ros::NodeHandle nh): nh_(nh){
    getParamOrThrow(nh_, "/lidar/point_interval", point_interval_);

    getParamOrThrow<double>(nh_, "/lidar/min_dist", min_dist_);
    getParamOrThrow<double>(nh_, "/lidar/max_dist", max_dist_);

    getParamOrThrow(nh_, "/lidar/min_intensity", min_intensity_);

    point_buffer_ = std::make_shared<StampedRingBuffer<RawLidarPoint>>(5.0 / point_interval_); // Keep track of at least 5 seconds worth of points
}


/*
Efficient parsing of pointcloud msg. Removing rough outliers on the fly. Emplacement into ringbuffer. 
PointCloud2 message is assumed structured such that each point is (x, y, z, intensity) where all are floating values.
*/
void Lidar::addFrame(sensor_msgs::PointCloud2::ConstPtr msg){
    double ts_point = msg->header.stamp.toSec() - point_interval_; // Header stamp is valid for the first point
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejection tests
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
        )
            continue;

        // Assert distance range
        double dist = sqrt(it[0]*it[0] + it[1]*it[1] + it[2]*it[2]);
        if (dist < min_dist_ || dist > max_dist_)
            continue;

        point_buffer_->addElement({
            ts_point,
            RawLidarPoint{
                it[0],
                it[1],
                it[2],
                it[3]
            }
        });        
    }
    
    ts_head_ = ts_point; // Only need to update this at end-of-frame. (All points inside a single frame is sequential)
}