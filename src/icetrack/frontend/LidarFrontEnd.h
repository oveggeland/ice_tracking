#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "backend/PoseGraph.h"

#include "frontend/PointBuffer.h"
#include "frontend/FrameBuffer.h"
#include "frontend/SurfaceEstimator.h"
#include "frontend/LidarOdometry.h"

#include "utils/ros_params.h"

/*
Main class of the front end. Most functionality is distributed to submodules which are frequently polled for updates upon arrival of new messages. 
*/
class LidarFrontEnd{
public:
    // Constructor
    LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph);

    // Interface
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Const accessors to front end resources
    const PointBuffer& pointBuffer();
    const FrameBuffer& frameBuffer();
    const SurfaceEstimator& surfaceEstimator();
    const LidarOdometry& lidarOdometry();

private:
    // Polls all submodules for updates
    void pollUpdates();

    // Buffer for incoming lidar points
    PointBuffer point_buffer_;

    // Buffer for maintaining pointcloud "frames"
    FrameBuffer frame_buffer_;

    // Estimate the ice sheet as under a plane assumption
    SurfaceEstimator surface_estimator_;

    // Frame-to-frame odometry
    LidarOdometry lidar_odometry_;
};
