/*
Entry point for icetrack node. This node use icenav and icemap to continously and create maps of the surrounding ice.
*/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

#include "icetrack/message_sequencer.h"
#include "icetrack/icetrack.h"

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "rosbag_icetrack_node");
    ros::NodeHandle nh;

    // Extract info parameters
    std::string bagpath; // Path to folder of bag files
    double safe_delay; // Safe delay for message sequencer

    nh.getParam("/bagpath", bagpath);
    nh.getParam("/safe_delay", safe_delay);

    // Initialize some navigation node
    IceTrack nav_object = IceTrack(nh, 60);

    // Initialize message sequencer
    MessageSequencer sequencer(nh, safe_delay);

    std::vector<std::filesystem::path> files;
    std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());

    // Iterate over bagfiles in chronological order
    for (const std::string& filename : files) {
        rosbag::Bag bag(filename);
        rosbag::View view(bag);

        for (rosbag::MessageInstance const m : view) {
            if (m.getDataType() == "sensor_msgs/Imu") {
                sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();

                auto imu_callback = std::function<void()>(std::bind(&IceTrack::imuCallback, &nav_object, msg));
                sequencer.pushCallback(msg->header.stamp, imu_callback);
            }
            else if (m.getDataType() == "sensor_msgs/NavSatFix") {
                sensor_msgs::NavSatFix::ConstPtr msg = m.instantiate<sensor_msgs::NavSatFix>();

                auto gnss_callback = std::function<void()>(std::bind(&IceTrack::gnssCallback, &nav_object, msg));
                sequencer.pushCallback(msg->header.stamp, gnss_callback);
            }
            else if (m.getDataType() == "sensor_msgs/PointCloud2") {
                sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

                auto pcl_callback = std::function<void()>(std::bind(&IceTrack::pclCallback, &nav_object, msg));
                sequencer.pushCallback(msg->header.stamp, pcl_callback);
            }

            // Check for "safe" callbacks at each iteration
            sequencer.pollCallbacks();

            if (!ros::ok()){
                sequencer.clear();
                break;
            }
        }
    }

    // Flush all remaining callbacks
    sequencer.flushCallbacks();

    return 0;
}