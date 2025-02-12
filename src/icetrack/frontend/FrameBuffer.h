#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>

#include "frontend/PointBuffer.h"
#include "backend/PoseGraph.h"

#include "utils/ros_params.h"


/*
Definition of a Lidar Frame. That is, all Lidar Points between two timesteps, typically undistorted by interpolation between pose graph estimates

- frame_idx: ID of frame. Corresponding to a pose idx in the pose graph. The frame is represented in this pose. 
- positions: Position vectors of each point, represented in pose_{frame_idx}
- intensities: The intensity values of each point. 
*/
struct FrameType{
    int frame_idx;
    Eigen::Matrix3Xf positions;
    Eigen::VectorXf intensities;

    // Constructor to enforce size allocation at initialization
    FrameType(int num_points)
        : positions(3, num_points), intensities(num_points) {}

    int size() const { return positions.cols(); }
};

using FrameBufferType = std::deque<FrameType>;
using FrameBufferConstIterator = FrameBufferType::const_iterator;

class FrameBuffer{
public:
    // Constructor
    FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer);
    
    // Interface
    void pollUpdates();

    // Accessors
    int getFirstFrameIdx() const { return buffer_.empty()? 0: buffer_.front().frame_idx; }
    int getLastFrameIdx() const { return buffer_.empty()? 0: buffer_.back().frame_idx; }
    int getPointCount() const { return point_count_; }

    bool hasFrame(int idx) const;
    const FrameType& getFrame(int idx) const;

    FrameBufferConstIterator begin() const { return buffer_.begin(); }
    FrameBufferConstIterator end() const { return buffer_.end(); }

private:
    FrameBufferType buffer_;
    int point_count_ = 0;

    // Buffer handling
    void createFrame(int idx); // Create a frame for specified idx
    FrameType& newFrame(int num_points); // Initialize a new frame in buffer and return reference
    void removeOldFrames(); // Remove out-of-scope frames

    // Configuration
    bool undistort_frames_;

    // Required resources
    const PointBuffer& point_buffer_; // Reference to buffer of incoming lidar points
    const PoseGraph& pose_graph_; // Reference to posegraph object
};
