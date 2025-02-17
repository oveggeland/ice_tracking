#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

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
    FrameBuffer(ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer);
    
    // Interface
    bool createFrame(int idx); // Create a frame for specified idx
    void removeOldFrames();
    void refineFrames();

    // Interval queries
    Eigen::Matrix3Xf getPoints() const { return getPointsWithin(getFirstTimeStamp(), getLastTimeStamp()); };
    Eigen::Matrix3Xf getPointsWithin(double t0, double t1) const;

    // Accessors
    double getFirstTimeStamp() const { return buffer_.empty()? 0.0: buffer_.front().t0(); }
    double getLastTimeStamp() const { return buffer_.empty()? 0.0: buffer_.back().t1(); }
    int getFirstFrameIdx() const { return buffer_.empty()? 0: buffer_.front().idx(); }
    int getLastFrameIdx() const { return buffer_.empty()? 0: buffer_.back().idx(); }

    const FrameType* getFrame(int idx) const;

    FrameBufferConstIterator begin() const { return buffer_.begin(); }
    FrameBufferConstIterator end() const { return buffer_.end(); }
    size_t size() const { return buffer_.size(); }

private:
    FrameBufferType buffer_;

    // 
    size_t cloud_size_ = 0;
    double window_size_ = 20;

    // Configuration
    bool undistort_frames_;

    // Required resources
    const PointBuffer& point_buffer_;   // Reference to buffer of incoming lidar points
    const PoseGraph& pose_graph_;       // Reference to PoseGraph for pose queries
};