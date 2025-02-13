#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include "frontend/PointBuffer.h"
#include "frontend/CloudFrame.h"

#include "backend/PoseGraph.h"

#include "utils/ros_params.h"


using FrameType = CloudFrame;
using FrameBufferType = std::deque<FrameType>;
using FrameBufferConstIterator = FrameBufferType::const_iterator;

class FrameBuffer{
public:
    // Constructor
    FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer);
    
    // Interface
    void pollUpdates();

    // Accessors
    int getFirstFrameIdx() const { return buffer_.empty()? 0: buffer_.front().idx(); }
    int getLastFrameIdx() const { return buffer_.empty()? 0: buffer_.back().idx(); }
    int getPointCount() const { return point_count_; }

    const FrameType* getFrame(int idx) const;

    FrameBufferConstIterator begin() const { return buffer_.begin(); }
    FrameBufferConstIterator end() const { return buffer_.end(); }

private:
    FrameBufferType buffer_;
    int point_count_ = 0;

    // Buffer handling
    void createFrame(int idx); // Create a frame for specified idx
    FrameType& addFrame(int idx, size_t capacity);
    void removeOldFrames();
    void refineFrames();

    // Configuration
    bool undistort_frames_;

    // Required resources
    const PointBuffer& point_buffer_; // Reference to buffer of incoming lidar points
    const PoseGraph& pose_graph_; // Reference to posegraph object
};
