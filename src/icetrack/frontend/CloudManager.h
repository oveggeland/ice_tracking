#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "backend/PoseGraph.h"

#include "frontend/PointBuffer.h"
#include "frontend/FrameBuffer.h"
#include "frontend/SurfaceEstimator.h"
#include "frontend/OdometryEstimator.h"
#include "frontend/CloudPublisher.h"

#include "utils/ros_params.h"

/*
Main class of the front end. Reactive logic based on pose and lidar callbacks. 
*/
class CloudManager {
public:
    // Constructor
    CloudManager(ros::NodeHandle& nh, PoseGraph& pose_graph);

    // Interface for events
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    // PoseGraph interface
    PoseGraph& pose_graph_;

    // Buffers 
    PointBuffer point_buffer_; // Buffer for incoming lidar points
    FrameBuffer frame_buffer_; // Buffer for maintaining pointcloud "frames"

    // Factor-generating modules
    SurfaceEstimator surface_estimator_; // Estimate the ice sheet as under a plane assumption
    OdometryEstimator odometry_estimator_; // Frame-to-frame odometry

    // Publisher(s)
    CloudPublisher cloud_publisher_;

    // Cloud data
    std::vector<Eigen::Vector3d> points_;
    std::vector<float> intensities_;

    // Functionality
    bool generateLidarFrame(const int state_idx);
    void refineFrames();
    void rebuildMap();

    // Config
    bool refine_frames_;
    bool publish_frames_;
    bool publish_cloud_;

    // Subscriber(s)
    ros::Subscriber lidar_sub_;
    ros::Subscriber pose_sub_;
};