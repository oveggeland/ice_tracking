#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include "icetrack/SensorSystem.h"
#include "icetrack/lidar.h"
#include "icetrack/file_system.h"

#include "icetrack/StampedRingBuffer.h"

struct PointDetailed{
    double x, y, z;
    float intensity;
    float intensity_corrected;
    float distance;
};

class CloudManager{
public:
    // Constructurs
    CloudManager(){};
    CloudManager(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors_);

    // Main entry from IceTrack
    void newPose(double ts, Pose3 T);

private:
    ros::NodeHandle nh_;
    
    // Cloud buffers
    std::shared_ptr<StampedRingBuffer<RawLidarPoint>> point_buffer_; // Incoming points, in LiDAR frame
    StampedRingBuffer<PointDetailed> cloud_;     // Templated stamped ringbuffer

    // For correction 
    double distance_ref_;

    // Keep track of previous pose
    double ts_prev_ = 0.0;
    Pose3 pose_prev_;

    // Keep track of offset
    double x0_ = 0.0;
    double y0_ = 0.0;
    Pose3 shiftPose(Pose3 pose);

    // Initializing
    bool init_ = false;
    bool isInit();
    void initialize(double t0, Pose3 pose0);

    // Bounds
    double z_lower_bound_, z_upper_bound_;

    // Cloud management and processing
    double window_size_; // Size of sliding window to use for analysis
    void analyseWindow();
    void calculateMoments();

    double window_interval_;
    double ts_analysis_ = 0.0;

    // Stats
    void writeStatistics();
    void saveElevBinary();
    std::ofstream f_stats_;

    int count_;
    double z_mean_, z_var_;

    double ts_exp_ = 0.0;
    double z_exp_mean_ = 0;
    double z_exp_var_ = 0;
    double exp_decay_rate_ = 0.1; // Time constant of 10 seconds

    // Saving
    bool save_cloud_;
    void saveCloud();

    // Paths
    std::string cloud_path_;
    std::string elev_path_; 
    std::string stats_path_;
};