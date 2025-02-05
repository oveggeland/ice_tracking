#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include "utils/calibration.h"
#include "utils/ros_params.h"
#include "utils/StampedRingBuffer.h"


struct RawLidarPoint {
    Eigen::Vector3f position;
    float intensity;
    double ts;
};


class LidarFrontEnd{
public:
    LidarFrontEnd(ros::NodeHandle& nh);

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    void buildFrame();
    void alignFrame();
    void estimatePlane();

    // Buffer for incoming lidar points
    StampedRingBuffer<RawLidarPoint> point_buffer_; // NB: Points are buffered in body-frame!

    inline bool acceptPoint(double ts, float x, float y, float z, float i) const;
    void parsePoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    // Extrinsic calibration (lidar->body)
    Eigen::Matrix3f bRl_;
    Eigen::Vector3f btl_;
};
