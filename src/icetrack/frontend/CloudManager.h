#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "backend/PoseGraph.h"

#include "frontend/PointBuffer.h"
#include "frontend/FrameBuffer.h"
#include "frontend/SurfaceEstimator.h"
#include "frontend/OdometryEstimator.h"
#include "frontend/CloudProcessor.h"
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

    // Const accessors to buffers
    const PointBuffer& pointBuffer() const { return point_buffer_; }
    const FrameBuffer& frameBuffer() const { return frame_buffer_; }

private:
    // PoseGraph interface
    PoseGraph& pose_graph_;

    // Buffers 
    PointBuffer point_buffer_; // Buffer for incoming lidar points
    FrameBuffer frame_buffer_; // Buffer for maintaining pointcloud "frames"

    // Factor-generating modules
    SurfaceEstimator surface_estimator_; // Estimate the ice sheet as under a plane assumption
    OdometryEstimator odometry_estimator_; // Frame-to-frame odometry

    // Processing modules
    CloudProcessor cloud_processor_; // Process raw pointclouds

    // Publisher(s)
    CloudPublisher cloud_publisher_;

    // Subscriber(s)
    ros::Subscriber lidar_sub_;
    ros::Subscriber pose_sub_;
};