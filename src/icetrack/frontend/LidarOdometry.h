#pragma once

#include <ros/ros.h>

#include "backend/PoseGraph.h"
#include "frontend/FrameBuffer.h"

#include "utils/pointcloud.h"
#include "utils/ros_params.h"

class LidarOdometry{
public: 
    // Constructor
    LidarOdometry(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const FrameBuffer& frame_buffer);

    // Interface
    void alignFrames() {};

private:
    const PoseGraph& pose_graph_;
    const FrameBuffer& frame_buffer_;
};