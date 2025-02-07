#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "frontend/PointBuffer.h"
#include "backend/PoseGraph.h"

#include "utils/calibration.h"
#include "utils/ros_params.h"
#include "utils/StampedRingBuffer.h"
#include "utils/pointcloud.h"

using FrameType = TensorCloudPtr;
using FrameBufferType = std::map<int, FrameType>;
using FrameBufferIterator = FrameBufferType::const_iterator;  // Read-only iterator

class FrameBuffer{
public:
    // Constructor
    FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer);
    
    // Interface
    void generateFrames();
    void addFrame(int idx);

    // Accessors
    int getIndex() const { return end()->first; }
    FrameType getFrame(int idx) const { 
        auto it = buffer_.find(idx);
        return (it != buffer_.end()) ? it->second : nullptr;
    }

    FrameBufferIterator begin() const { return buffer_.begin(); }
    FrameBufferIterator end() const { return buffer_.end(); }
    FrameBufferIterator lowerBound(int idx) const { return buffer_.lower_bound(idx); }

private:
    FrameBufferType buffer_;

    // Keep track of latest count
    int frame_count_ = 0;

    bool undistort_ = false;
    bool visualize_ = false;
    int visualize_count_ = 5000;

    // Access
    const PointBuffer& point_buffer_;
    const PoseGraph& pose_graph_;
};
