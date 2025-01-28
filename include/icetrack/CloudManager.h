#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <open3d/Open3D.h>
#include <open3d/core/Tensor.h>
#include <open3d/t/geometry/PointCloud.h>

#include <gtsam/geometry/Pose3.h>

#include "icetrack/system/SensorSystem.h"
#include "icetrack/utils/ros_params.h"
#include "icetrack/utils/pointcloud.h"

#include "icetrack/utils/StampedRingBuffer.h"
#include "icetrack/utils/file_system.h"
#include "icetrack/utils/calibration.h"
#include "icetrack/utils/conversions.h"
#include "icetrack/utils/CallbackSequencer.h"

class CloudManager{
public:
    CloudManager(ros::NodeHandle nh);

private:
    bool enabled_;
    
    // Message sequencing and subscriber callbacks
    CallbackSequencer sequencer_;

    ros::Subscriber lidar_sub_;
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    ros::Subscriber pose_sub_;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseSafeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    // Extrinsics (lidar->imu)
    gtsam::Pose3 bTl_;
    
    // Cloud buffers
    StampedRingBuffer<PointXYZIT> point_buffer_; // Incoming points, in LiDAR frame
    double point_interval_ = 5.0e-6;
    double ts_head_ = 0.0;
    double min_dist_squared_ = 15*15;
    double max_dist_squared_ = 100*100;
    
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