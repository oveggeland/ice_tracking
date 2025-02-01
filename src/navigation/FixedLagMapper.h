#pragma once

#include "ros/ros.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "LidarBuffer.h"
#include "PoseGraphManager.h"


// TODO: Apply this structure to the system
// class FixedLagMapper {
// public:
//     FixedLagMapper(ros::NodeHandle nh);

//     void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
//     void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
//     void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

// private:
//     // Shared resources
//     PointBuffer point_buffer_;
//     FixedLagFrameBuffer frame_buffer_ (point_buffer_);

//     PoseGraphManager pose_graph_manager_;                // This is essentially PoseEstimator


//     const Smoother& smoother_;                           // We have a reference to the pose graph smoother. 
// };
/*
Flow is:

On lidar message, add to point buffer.

On IMU or GNSS message, check return type from PoseGraphManager, indicating a new state is added.
If a new state is added, generate a Frame. Check then if it is time to publish a new cloud.

Whenever 
*/

class FixedLagMapper{
public:
    FixedLagMapper(ros::NodeHandle& nh);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

private:
    LidarBuffer lidar_buffer_;
    //     FixedLagFrameBuffer frame_buffer_ (point_buffer_);

    // Pose optimization
    PoseGraphManager pose_graph_manager_;
    const BatchFixedLagSmoother& pose_graph_; // Reference to pose graph smoother

    void newPose(int pose_idx);
//     PointBuffer point_buffer_;
//     FixedLagFrameBuffer frame_buffer_ (point_buffer_);

//     PoseGraphManager pose_graph_manager_;                // This is essentially PoseEstimator


//     const Smoother& smoother_;                           // We have a reference to the pose graph smoother. 

};