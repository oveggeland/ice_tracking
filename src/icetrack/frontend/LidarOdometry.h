#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/t/pipelines/registration/TransformationEstimation.h>

#include "backend/PoseGraph.h"
#include "frontend/FrameBuffer.h"

#include "utils/pointcloud.h"
#include "utils/ros_params.h"

class LidarOdometry{
public: 
    // Constructor
    LidarOdometry(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer);

    // Interface
    void pollUpdates();

private:
    PoseGraph& pose_graph_;
    const FrameBuffer& frame_buffer_;

    void alignFrames(int idx0, int idx1);

    // Control
    int aligned_idx_ = 0;

    // Config
    int frame_interval_; // Align every second frame
    int min_frame_size_;
    double voxel_size_;
    double icp_threshold_;
    double icp_min_fitness_;

};