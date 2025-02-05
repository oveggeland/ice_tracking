#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "PoseGraph.h"

#include "PointBuffer.h"
#include "FrameBuffer.h"

#include "SurfaceEstimator.h"

#include "utils/ros_params.h"



class LidarFrontEnd{
public:
    LidarFrontEnd(ros::NodeHandle& nh, PoseGraph& pose_graph);

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    PoseGraph& pose_graph_;

    PointBuffer point_buffer_;
    FrameBuffer frame_buffer_;

    SurfaceEstimator surface_estimator_;
};
