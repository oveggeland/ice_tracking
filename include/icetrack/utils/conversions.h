/*
Header library for conversions between GTSAM and ROS geometries.
*/
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Pose3.h>
#include <geometry_msgs/Pose.h>

inline gtsam::Point3 translationRosToGtsam(const geometry_msgs::Point& msg){
    return gtsam::Point3(msg.x, msg.y, msg.z);
}

inline geometry_msgs::Point translationGtsamToRos(const gtsam::Point3& p){
    geometry_msgs::Point msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();

    return msg;
}

inline gtsam::Rot3 rotationRosToGtsam(const geometry_msgs::Quaternion& msg){
    return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z);
}

inline geometry_msgs::Quaternion rotationGtsamToRos(const gtsam::Rot3& rot){
    gtsam::Quaternion quaternion = rot.toQuaternion();

    geometry_msgs::Quaternion msg;
    msg.w = quaternion.w();
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();

    return msg;
}

inline gtsam::Pose3 poseRosToGtsam(const geometry_msgs::Pose& msg){
    return gtsam::Pose3(
        rotationRosToGtsam(msg.orientation),
        translationRosToGtsam(msg.position)
    );
}

inline geometry_msgs::Pose poseGtsamToRos(const gtsam::Pose3& pose){
    geometry_msgs::Pose msg;
    msg.position = translationGtsamToRos(pose.translation());
    msg.orientation = rotationGtsamToRos(pose.rotation());

    return msg;
}