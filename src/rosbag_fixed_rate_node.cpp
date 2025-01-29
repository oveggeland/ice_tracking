#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <rosgraph_msgs/Clock.h>

#include <filesystem>

#include "utils/ros_params.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_icetrack_node");
    ros::NodeHandle nh;

    std::string bagpath = getParamOrThrow<std::string>(nh, "/bagpath");
    std::string imu_topic = getParamOrThrow<std::string>(nh, "/imu_topic");
    std::string gnss_topic = getParamOrThrow<std::string>(nh, "/gnss_topic");
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");
    double max_publish_rate = getParamOrDefault<double>(nh, "/max_publish_rate", 1.0);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 2000);
    ros::Publisher gnss_pub = nh.advertise<sensor_msgs::NavSatFix>(gnss_topic, 10);
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic, 100);
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    std::vector<std::filesystem::path> files;
    std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());

    ros::Time last_msg_time = ros::Time(0);

    for (const auto& filepath : files) {
        std::string filename = filepath.string();
        ROS_INFO_STREAM("Reading bag: " << filename);

        rosbag::Bag bag;
        try {
            bag.open(filename, rosbag::bagmode::Read);
        } catch (const rosbag::BagException& e) {
            ROS_ERROR_STREAM("Failed to open bag file: " << filename << ". Error: " << e.what());
            continue;
        }

        rosbag::View view(bag);
        for (rosbag::MessageInstance const m : view) {
            ros::Time msg_time = m.getTime();

            if (max_publish_rate > 0.0 && last_msg_time.toSec() > 0) {
                ros::Duration delay = msg_time - last_msg_time;
                if (delay.toSec() > 0) {
                    ros::Duration sleep_time = delay * (1.0 / max_publish_rate);
                    sleep_time.sleep();
                }
            }
            last_msg_time = msg_time;

            rosgraph_msgs::Clock clock_msg;
            clock_msg.clock = msg_time;
            clock_pub.publish(clock_msg);

            if (m.getDataType() == "sensor_msgs/Imu") {
                auto msg = m.instantiate<sensor_msgs::Imu>();
                if (msg) imu_pub.publish(msg);
            } else if (m.getDataType() == "sensor_msgs/NavSatFix") {
                auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                if (msg) gnss_pub.publish(msg);
            } else if (m.getDataType() == "sensor_msgs/PointCloud2") {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (msg) lidar_pub.publish(msg);
            }

            ros::spinOnce();
            if (!ros::ok()) break;
        }
        bag.close();
        if (!ros::ok()) break;
    }

    return 0;
}