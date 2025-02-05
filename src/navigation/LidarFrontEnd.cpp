#include "LidarFrontEnd.h"


LidarFrontEnd::LidarFrontEnd(ros::NodeHandle& nh){
    // Initialize point buffer
    double buffer_size = getParamOrThrow<double>(nh, "/navigation/surface_estimation/buffer_size");
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_point_interval", point_interval_);
    point_buffer_ = StampedRingBuffer<RawLidarPoint>(buffer_size / point_interval_);

    // Extrinsic calibration
    gtsam::Pose3 bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file")); // TODO: Make cleaner
    bRl_ = bTl_.rotation().matrix().cast<float>();
    btl_ = bTl_.translation().cast<float>(); 

    // Point filtering configuration
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_max_dist"), 2);
}


void LidarFrontEnd::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // Add new points to buffer
    parsePoints(msg);

    // Try to build new frame
    buildFrame();
}


inline bool LidarFrontEnd::acceptPoint(double ts, float x, float y, float z, float i) const{
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

void LidarFrontEnd::parsePoints(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    double ts_point = msg->header.stamp.toSec(); // Initial timestamp (assumed fixed point_interval_ between poinst)

    // Iterate through all points (Assume points on format: float x, float y, float z, float intensity)
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        if (acceptPoint(ts_point, it[0], it[1], it[2], it[3])) {
            Eigen::Vector3f point_imu = bRl_ * Eigen::Vector3f(it[0], it[1], it[2]) + btl_; // Transform to IMU frame
            point_buffer_.addPoint({ point_imu, it[3], ts_point });
        }
        ts_point += point_interval_;
    }

    ts_head_ = std::max(ts_head_, ts_point);
}


void LidarFrontEnd::buildFrame(){
    // TODO: Build frame if new pose is available


    alignFrame();
    estimatePlane();
}


void LidarFrontEnd::alignFrame(){
    // TODO: Lidar odometry
}


void LidarFrontEnd::estimatePlane(){

}