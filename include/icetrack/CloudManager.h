#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include <gtsam/geometry/Pose3.h>

#include "icetrack/system/SensorSystem.h"
#include "icetrack/utils/utils.h"

#include "icetrack/utils/StampedRingBuffer.h"

struct PointDetailed{
    double ts;
    double x, y, z;
    float intensity;
};

class CloudManager{
public:
    CloudManager(ros::NodeHandle nh, const SensorSystem& sensors_);
    ~CloudManager();

    // Main entry from IceTrack
    void newPose(double t1, gtsam::Pose3 imu_pose);

private:
    // Extrinsics (lidar->imu)
    gtsam::Pose3 bTl_;
    
    // Cloud buffers
    const StampedRingBuffer<RawLidarPoint>& point_buffer_; // Incoming points, in LiDAR frame
    
    // New way of tracking points
    std::vector<double> positions_;
    std::vector<float> intensities_;

    // Keep track of previous pose
    double t0_ = 0.0;
    gtsam::Pose3 pose0_;

    // Keep track of offset
    double x0_ = 0.0;
    double y0_ = 0.0;
    gtsam::Pose3 shiftPose(gtsam::Pose3 pose);

    // Initializing
    bool init_ = false;
    bool isInit();
    void initialize(double t0, gtsam::Pose3 pose0);

    // Filtering
    double z_lower_bound_, z_upper_bound_;

    // Cloud management and processing
    double t0_window_;
    double window_size_;

    void analyseWindow();

    // Stats
    int count_;
    double z_mean_, z_var_;

    double ts_exp_ = 0.0;
    double z_exp_mean_ = 0;
    double z_exp_var_ = 0;
    double exp_decay_rate_ = 0.1; // Time constant of 10 seconds

    std::ofstream f_stats_;

    void writeStatistics();
   
    // Saving
    bool save_cloud_;
    void saveCloud(const open3d::t::geometry::PointCloud& pcd);

    // Paths
    std::string cloud_path_;
    std::string stats_path_;
};