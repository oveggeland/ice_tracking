#include "PointBuffer.h"

PointBuffer::PointBuffer(const ros::NodeHandle& nh){
    // Initialize point buffer
    double buffer_size = getParamOrThrow<double>(nh, "/point_buffer/buffer_size");
    getParamOrThrow(nh, "/lidar/point_interval", point_interval_);
    buffer_ = PointBufferType(buffer_size / point_interval_);

    // Extrinsic calibration
    gtsam::Pose3 bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file")); // TODO: Make cleaner
    bRl_ = bTl_.rotation().matrix().cast<float>();
    btl_ = bTl_.translation().cast<float>(); 

    // Point filtering configuration
    getParamOrThrow(nh, "/lidar/min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/lidar/min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/lidar/max_dist"), 2);
};


void PointBuffer::addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts_point = msg->header.stamp.toSec(); // Initial timestamp (assumed fixed point_interval_ between poinst)

    // Iterate through all points (Assume points on format: float x, float y, float z, float intensity)
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        if (acceptPoint(ts_point, it[0], it[1], it[2], it[3])) {
            Eigen::Vector3f point_imu = bRl_ * Eigen::Vector3f(it[0], it[1], it[2]) + btl_; // Transform to IMU frame
            buffer_.addPoint({
                point_imu.x(), 
                point_imu.y(), 
                point_imu.z(), 
                it[3], 
                ts_point
            });
        }
        ts_point += point_interval_;
    }

    ts_head_ = std::max(ts_head_, ts_point);
}

inline bool PointBuffer::acceptPoint(double ts, float x, float y, float z, float i) const{
    // Ensure chronological order of timestamps (if necessary)
    if (ts < ts_head_) 
        return false;

    // Check for invalid point or low intensity
    if (x == 0.0f || i < min_intensity_) 
        return false;

    // Compute squared distance once
    float dist_squared = x * x + y * y + z * z;
    if (dist_squared < min_dist_squared_ || dist_squared > max_dist_squared_) 
        return false;

    return true;
}

std::shared_ptr<open3d::geometry::PointCloud> PointBuffer::getCloud(const double t0, const double t1) const{
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    const auto it0 = lowerBound(t0);
    const auto it1 = lowerBound(t1);
    const int num_points = it0.distance_to(it1);
    cloud->points_.reserve(num_points);

    for (auto it = it0; it != it1; ++it){
        cloud->points_.emplace_back(it->x, it->y, it->z);
    }

    return cloud;
}