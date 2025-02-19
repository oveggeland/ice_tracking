#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <rosgraph_msgs/Clock.h>

#include <filesystem>
#include <gperftools/profiler.h>

#include "icetrack/utils/ros_params.h"
#include "icetrack/FixedLagMapper.h"

int main(int argc, char** argv) {
    // Initialize node
    ros::init(argc, argv, "rosbag_icetrack_node");
    ros::NodeHandle nh;

    // Start profiler
    std::string ws = getParamOrThrow<std::string>(nh, "/workspace");
    std::string exp = getParamOrThrow<std::string>(nh, "/exp");
    std::string profile_path = joinPaths({ws, exp, "rosbag_node.profile"});
    makePath(profile_path);
    ProfilerStart(profile_path.c_str());

    // Get parameters for the bag path and topic names
    std::string bagpath = joinPaths({ws, "bags"});
    std::string imu_topic = getParamOrThrow<std::string>(nh, "/imu_topic");
    std::string gnss_topic = getParamOrThrow<std::string>(nh, "/gnss_topic");
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");
    std::string image_topic = getParamOrThrow<std::string>(nh, "/image_topic"); // New Image Topic

    // Publishers for IMU, GNSS, PointCloud2, Image, and simulated clock
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 10);
    ros::Publisher gnss_pub = nh.advertise<sensor_msgs::NavSatFix>(gnss_topic, 10);
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic, 10);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>(image_topic, 10); // New Image Publisher
    ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // Initialize nodes
    FixedLagMapper fixed_lag_mapper(nh);

    // Collect bag files
    std::vector<std::filesystem::path> files;
    std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());

    // Process each bag file
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

        // Process each message in the bag
        for (rosbag::MessageInstance const m : view) {
            // Publish clock time
            rosgraph_msgs::Clock clock_msg;
            clock_msg.clock = m.getTime();
            clock_pub.publish(clock_msg);

            // Publish the message to the appropriate topic
            if (m.getDataType() == "sensor_msgs/Imu") {
                auto msg = m.instantiate<sensor_msgs::Imu>();
                if (msg) imu_pub.publish(msg);

            } else if (m.getDataType() == "sensor_msgs/NavSatFix") {
                auto msg = m.instantiate<sensor_msgs::NavSatFix>();
                if (msg) gnss_pub.publish(msg);

            } else if (m.getDataType() == "sensor_msgs/PointCloud2") {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (msg) lidar_pub.publish(msg);

            } else if (m.getDataType() == "sensor_msgs/Image") {  // Handling image messages
                auto msg = m.instantiate<sensor_msgs::Image>();
                if (msg) image_pub.publish(msg);
            }

            // Allow nodes to process messages
            ros::spinOnce();

            // Check if ROS is still running
            if (!ros::ok())
                break;
        }

        bag.close();

        // Check if ROS is still running
        if (!ros::ok())
            break;
    }

    ProfilerStop();
    return 0;
}
