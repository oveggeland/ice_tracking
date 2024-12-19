#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <filesystem>

#include "icetrack/icetrack.h"

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "rosbag_icetrack_node");
    ros::NodeHandle nh;

    // Extract info parameters
    std::string bagpath = getParamOrThrow<std::string>(nh, "/bagpath");

    // Initialize some navigation node
    IceTrack tracker = IceTrack(nh);

    std::vector<std::filesystem::path> files;
    std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());

    // Iterate over bagfiles in chronological order
    std::vector<std::string> topic_types = {"sensor_msgs/Imu", "sensor_msgs/NavSatFix", "sensor_msgs/PointCloud2"};
    for (const std::string& filename : files) {
        ROS_INFO_STREAM("Reading bag: " << filename);
        rosbag::Bag bag(filename);
        rosbag::View view(bag);//, rosbag::TopicQuery(topic_types));

        for (rosbag::MessageInstance const m : view) {
            if (m.getDataType() == "sensor_msgs/Imu") {
                sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();

                tracker.imuCallback(msg);
            }
            else if (m.getDataType() == "sensor_msgs/NavSatFix") {
                sensor_msgs::NavSatFix::ConstPtr msg = m.instantiate<sensor_msgs::NavSatFix>();
                
                tracker.gnssCallback(msg);
            }
            else if (m.getDataType() == "sensor_msgs/PointCloud2") {
                sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

                tracker.pclCallback(msg);
            }

            if (!ros::ok()){
                break;
            }
        }
    }
    
    return 0;
}