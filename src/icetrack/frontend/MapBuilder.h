#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "backend/PoseGraph.h"
#include "frontend/FrameBuffer.h"

#include <Eigen/Dense>
#include <open3d/Open3D.h>

#include "utils/ros_params.h"

class MapBuilder{
public:
    MapBuilder(ros::NodeHandle& nh, const PoseGraph& pose_graph, const FrameBuffer& frame_buffer);

    void pollUpdates();
private:
    const PoseGraph& pose_graph_;
    const FrameBuffer& frame_buffer_;

    double ts_map_;
    Eigen::MatrixXd map_;

    int last_frame_idx_ = 0;
    void updateMap();

    void visualizeMap();

    // Publishing
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_msg_;

    void initializeCloudMsg(const ros::NodeHandle& nh);
    void publishAsPointCloud2();
};

