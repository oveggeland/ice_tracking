#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <gtsam/geometry/Pose3.h>

#include "frontend/PointBuffer.h"
#include "frontend/CloudFrame.h"

#include "backend/PoseGraph.h"

#include "utils/ros_params.h"
#include "utils/pointcloud.h"


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

    open3d::t::geometry::PointCloud buildMap() const;

    // Query frames with specified attributes. If timestamps are not specified, all points are returned.
    // TensorCloud getTensorCloud() const; // Return t::geometry::pointcloud object with global positions and intensities
    // CloudFrame::Ptr getPoints(bool local, bool global, bool intensities, bool timestamps) const; // Get all points
    // CloudFrame::Ptr getPoints(double t0, double t1, bool local, bool global, bool intensities, bool timestamps) const; // Get points within time window

    // Accessors
    double t0() const { return buffer_.empty()? 0.0: buffer_.front().t0(); }
    double t1() const { return buffer_.empty()? 0.0: buffer_.back().t1(); }
    size_t pointCount() const { return point_count_; }

    const FrameType* getFrame(int idx) const;

    const FrameType& front() const { return buffer_.front(); }
    const FrameType& back() const { return buffer_.back(); }
    FrameBufferConstIterator begin() const { return buffer_.begin(); }
    FrameBufferConstIterator end() const { return buffer_.end(); }
    size_t size() const { return buffer_.size(); }

private:
    FrameBufferType buffer_;        // Frame buffer
    size_t point_count_ = 0;        // Total number of points (in all frames)

    // Configuration
    double window_size_;
    bool undistort_frames_;

    // Required resources
    const PointBuffer& point_buffer_;   // Reference to buffer of incoming lidar points
    const PoseGraph& pose_graph_;       // Reference to PoseGraph for pose queries
};