#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/StampedRingBuffer.h"

// Some useful naming
using PointCloud = open3d::geometry::PointCloud;
using PointCloudSharedPtr = std::shared_ptr<PointCloud>;
using FrameBuffer = std::map<int, PointCloudSharedPtr>;

using PointType = PointXYZT;
using PointBuffer = StampedRingBuffer<PointType>;
using PointBufferIterator = StampedRingBufferIterator<PointType>;

// Forward declaration
class PoseGraphManager;

class CloudManager{
public: 
    // Initialize
    CloudManager(ros::NodeHandle& nh);
    void setPoseGraphManager(PoseGraphManager& pose_graph_manager) {pose_graph_manager_ = &pose_graph_manager; }

    // Callback
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        addPoints(msg);
    };

    // Interface (stuff called from Navigation module)
    void newState(int idx){
        createFrame(idx);
    };
    const PointBufferIterator pointIteratorLowerBound(double ts) const { return point_buffer_.iteratorLowerBound(ts); }

private:
    PoseGraphManager* pose_graph_manager_;

    // Buffers
    PointBuffer point_buffer_;
    FrameBuffer frame_buffer_;

    // Modify and access buffers
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void createFrame(int state_idx);

    // Calibration matrix (lidar->imu)
    gtsam::Pose3 bTl_;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;
};