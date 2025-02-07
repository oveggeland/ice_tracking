#pragma once

#include <ros/ros.h>

#include "backend/PoseGraph.h"
#include "frontend/FrameBuffer.h"

#include "utils/pointcloud.h"
#include "utils/ros_params.h"

class LidarOdometry{
public: 
    // Constructor
    LidarOdometry(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer);

    // Interface
    void alignFrames();

private:
    PoseGraph& pose_graph_;
    const FrameBuffer& frame_buffer_;

    void alignFrame(int idx0, int idx1);

    // Control
    int aligned_idx_ = 0;
    int frame_interval_ = 2; // Align every second frame

};