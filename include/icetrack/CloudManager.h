#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include <gtsam/geometry/Pose3.h>

#include "icetrack/system/SensorSystem.h"
#include "icetrack/utils/utils.h"
#include "icetrack/utils/pointcloud.h"

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
    bool enabled_;
    
    // Extrinsics (lidar->imu)
    gtsam::Pose3 bTl_;
    
    // Cloud buffers
    const StampedRingBuffer<RawLidarPoint>& point_buffer_; // Incoming points, in LiDAR frame
    
    // New way of tracking points
    std::vector<float> positions_;
    std::vector<uint8_t> intensities_;

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
    double min_elevation_;
    double max_elevation_;
    float min_intensity_;

    double grid_size_;
    double smoothing_radius_;
    double deformation_radius_;


    // Cloud management and processing
    double t0_window_;
    double window_size_;

    void analyseWindow();
    void processCloud(open3d::t::geometry::PointCloud& pcd);
    void saveCloud(const open3d::t::geometry::PointCloud& pcd);


    std::ofstream f_stats_;
   
    // Saving
    bool save_cloud_;

    // Paths
    std::string cloud_path_;
    std::string stats_path_;
};