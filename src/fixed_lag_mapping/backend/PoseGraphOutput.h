#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "backend/navigation.h"

#include "utils/ros_params.h"
#include "utils/file_system.h"
#include "utils/conversions.h"


// Abstraction to deal with ROS publication of tf and pose, and saving to file
class PoseGraphOutput {
public:
    PoseGraphOutput(ros::NodeHandle& nh);

    void outputState(const PoseGraphState& state);
private:
    // TF broadcasting
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped tf_;
    void setupBroadcaster(const ros::NodeHandle& nh);
    void broadcastTransform(const PoseGraphState& state);

    // Ros publisher
    ros::Publisher pose_pub_;
    geometry_msgs::PoseStamped pose_msg_;
    void setupPublisher(ros::NodeHandle& nh);
    void publishPose(const PoseGraphState& state);

    // File stream
    std::ofstream f_out_;
    void setupFileStream(const ros::NodeHandle& nh);
    void writeToFile(const PoseGraphState& state);
};