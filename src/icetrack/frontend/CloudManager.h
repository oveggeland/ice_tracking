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

    // Keep track of raw and processed clouds
    open3d::t::geometry::PointCloud raw_cloud_;
    open3d::geometry::PointCloud cloud_;

    // Config
    bool publish_cloud_;
    double ts_last_cloud_pub_ = 0.0;
    double cloud_pub_interval_;

    bool publish_frames_;

    bool generateLidarFrame(const int state_idx);
    void refineFrames();
    void rebuildMap();


    // Subscriber(s)
    ros::Subscriber lidar_sub_;
    ros::Subscriber pose_sub_;
};