#pragma once

#include <deque>
#include <ros/ros.h>

#include "LidarFrame.h"
#include "utils/ros_params.h"

using FrameBufferType = std::deque<LidarFrame>;
using FrameBufferIterator = FrameBufferType::iterator;
using FrameBufferConstIterator = FrameBufferType::const_iterator;

class FrameBuffer{
public:
    // Constructor
    FrameBuffer(const ros::NodeHandle& nh);
    
    // Emplace LidarFrame in place and return a reference
    LidarFrame& emplaceFrame(const int frame_id, const double ts, const size_t capacity);
    void maintain();

    // Accessors
    size_t numFrames() const { return buffer_.size(); }
    size_t numPoints() const { return num_points_; }
    
    LidarFrame* getFrame(const int frame_id);
    const LidarFrame* getFrame(const int frame_id) const;

    FrameBufferIterator begin() { return buffer_.begin(); }
    FrameBufferIterator end() { return buffer_.end(); }

    FrameBufferConstIterator cbegin() const { return buffer_.cbegin(); }
    FrameBufferConstIterator cend() const { return buffer_.cend(); }

private:
    FrameBufferType buffer_;       // Main buffer
    size_t num_points_ = 0;        // Total number of points (in all frames)

    // Config
    double window_size_;            // Only keep frames in fixed window
};