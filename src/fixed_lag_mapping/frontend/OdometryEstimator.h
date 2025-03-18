#pragma once

#include <ros/ros.h>

#include <open3d/Open3D.h>
#include <open3d/t/pipelines/registration/TransformationEstimation.h>

#include "backend/PoseGraph.h"
#include "frontend/FrameBuffer.h"

#include "utils/pointcloud.h"
#include "utils/ros_params.h"

class OdometryEstimator{
public: 
    // Constructor
    OdometryEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer);

    // Interface
    void estimateOdometry(int idx1);

private:
    PoseGraph& pose_graph_;
    const FrameBuffer& frame_buffer_;

    // Keep track of previous key frame
    int prev_idx_ = 0;
    std::shared_ptr<PointCloud> prev_cloud_ = nullptr;

    // Config
    bool enabled_;
    int key_frame_interval_; // Only align key frames
    int min_frame_size_;
    double voxel_size_;
    double icp_threshold_;
    double icp_min_fitness_;
    int icp_max_iter_;
    double icp_relative_rmse_;
    double icp_relative_fitness_;

};