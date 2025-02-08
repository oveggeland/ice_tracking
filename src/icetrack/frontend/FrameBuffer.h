#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <open3d/t/geometry/PointCloud.h>

#include "frontend/PointBuffer.h"
#include "backend/PoseGraph.h"

#include "utils/ros_params.h"
#include "utils/pointcloud.h"

using FrameType = TensorCloudPtr;
using FrameBufferType = std::map<int, FrameType>;
using FrameBufferIterator = FrameBufferType::const_iterator;  // Read-only iterator

class FrameBuffer{
public:
    // Constructor
    FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer);
    
    // Interface
    void pollUpdates();

    // Accessors
    const FrameType getFrame(int idx) const { 
        auto it = buffer_.find(idx);
        return (it != buffer_.end()) ? it->second : nullptr;
    }

    FrameBufferIterator begin() const { return buffer_.begin(); }
    FrameBufferIterator end() const { return buffer_.end(); }
    FrameBufferIterator lowerBound(int idx) const { return buffer_.lower_bound(idx); }

private:
    FrameBufferType buffer_;

    // Add frame with points between t_{idx-1} and t_{idx}
    void addFrame(int idx);

    // Remove out of scope frames
    void removeOldFrames();

    // Option to undistort frames by pose interpolation
    bool undistort_frames_;

    // Reference to buffer of incoming lidar points
    const PointBuffer& point_buffer_;

    // Reference to posegraph object
    const PoseGraph& pose_graph_;
};
