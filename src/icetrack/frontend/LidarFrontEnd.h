#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "backend/PoseGraph.h"

#include "frontend/PointBuffer.h"
#include "frontend/FrameBuffer.h"
#include "frontend/SurfaceEstimator.h"
#include "frontend/LidarOdometry.h"

#include "utils/ros_params.h"



class LidarFrontEnd{
public:
    // Constructor
    LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph);

    // Interface
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void pollUpdates();

private:
    PoseGraph& pose_graph_;

    PointBuffer point_buffer_;
    FrameBuffer frame_buffer_;

    SurfaceEstimator surface_estimator_;
    LidarOdometry lidar_odometry_;
    
    // Actions
    double ts_plane_; // Timestamp of last fitted plane
    void planeFitting();
    
    void odometry(int state_idx) {} // TODO
};
