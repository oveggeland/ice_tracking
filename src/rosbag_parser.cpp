/*
This node reads a series of bagfiles and publishes the messages in chronological order.
A safe time to wait before publishing is used to ensure that no prior messages are lagging behind.
*/

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <algorithm>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>


class RosbagParser{
public:
    RosbagParser(ros::NodeHandle nh, std::filesystem::path bagpath, double safe_delay, std::vector<std::string> topics):
        safe_delay_(safe_delay){
            imu_pub_ = nh.advertise<sensor_msgs::Imu>(topics[0], 100);
            gnss_pub_ = nh.advertise<sensor_msgs::NavSatFix>(topics[1], 10);
            pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topics[2], 10);

            parseFiles(bagpath);
        }

private:
    ros::Time t_wall_;
    ros::Time t_head_;
    ros::Time t_safe_;
    double safe_delay_;

    std::map<ros::Time, std::tuple<std::shared_ptr<void>, std::string>> message_buffer_;

    ros::Publisher imu_pub_;
    ros::Publisher gnss_pub_;
    ros::Publisher pcl_pub_;

    void parseMessage(std::tuple<std::shared_ptr<void>, std::string> msg_tuple){
        std::string topic = std::get<1>(msg_tuple);
        
        if (topic == "sensor_msgs/Imu"){
            std::shared_ptr<sensor_msgs::Imu> msg = std::static_pointer_cast<sensor_msgs::Imu>(std::get<0>(msg_tuple));
            imu_pub_.publish(*msg);
        }
        else if (topic == "sensor_msgs/NavSatFix"){
            std::shared_ptr<sensor_msgs::NavSatFix> msg = std::static_pointer_cast<sensor_msgs::NavSatFix>(std::get<0>(msg_tuple));
            gnss_pub_.publish(*msg);
        }
        else if (topic == "sensor_msgs/PointCloud2"){
            std::shared_ptr<sensor_msgs::PointCloud2> msg = std::static_pointer_cast<sensor_msgs::PointCloud2>(std::get<0>(msg_tuple));
            pcl_pub_.publish(*msg);
        }
    }

    void parseMessageBuffer(){
        for (auto msg = message_buffer_.begin(); msg != message_buffer_.end();){
            ros::Time ts = msg->first;
            if (ts < t_safe_){
                if (ts < t_head_){
                    ROS_WARN_STREAM("Timestamp before filter head: " << std::get<1>(msg->second));
                }
                else{
                    parseMessage(msg->second);
                    t_head_ = ts;
                }

                message_buffer_.erase(msg++);
            }
            else{
                ++msg;
            }
        }
    }

    void parseBag(const std::filesystem::path& bagpath) {
        ROS_INFO_STREAM("Reading bag: " << bagpath);
        rosbag::Bag bag(bagpath.string());

        // Create a view of the bag that contains all the messages
        rosbag::View view(bag);

        // Iterate through all the messages in the bag
        for (const rosbag::MessageInstance& msgInstance : view) {
            ros::Time ts;
            std::shared_ptr<void> message_ptr;
            std::string data_type = msgInstance.getDataType();
            
            if (data_type == "sensor_msgs/Imu") {
                sensor_msgs::Imu::ConstPtr imuMsg = msgInstance.instantiate<sensor_msgs::Imu>();
                message_ptr = std::make_shared<sensor_msgs::Imu>(*imuMsg);
                ts = imuMsg->header.stamp;
            }    
            else if (data_type == "sensor_msgs/NavSatFix"){
                sensor_msgs::NavSatFix::ConstPtr gnssMsg = msgInstance.instantiate<sensor_msgs::NavSatFix>();
                message_ptr = std::make_shared<sensor_msgs::NavSatFix>(*gnssMsg);
                ts = gnssMsg->header.stamp;
            }
            else if (data_type == "sensor_msgs/PointCloud2"){
                sensor_msgs::PointCloud2::ConstPtr pclMsg = msgInstance.instantiate<sensor_msgs::PointCloud2>();
                message_ptr = std::make_shared<sensor_msgs::PointCloud2>(*pclMsg);
                ts = pclMsg->header.stamp;
            }
            else{
                continue; // Skip to next message
            }

            if (ts > t_wall_){
                t_wall_ = ts;
                t_safe_ = t_wall_ - ros::Duration(safe_delay_);
            }
            message_buffer_[ts] = std::make_tuple(message_ptr, data_type);
            parseMessageBuffer();
        }

        bag.close();  // Close the bag after processing
    }

    void parseFiles(const std::filesystem::path bagpath){
        std::vector<std::filesystem::path> files;
        std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
        std::sort(files.begin(), files.end());

        for (const std::string& filename : files) {
            parseBag(filename);
        }

        // Parse remaining messages
        t_safe_ = t_wall_ + ros::Duration(1.0e-9);
        parseMessageBuffer();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_parsing_node");
    ros::NodeHandle nh("~");

    std::string bagpath; // Path to folder of bag files
    if (!nh.getParam("/bagpath", bagpath)){
        ROS_ERROR("Failed to retrieve bag path.");
        return 1;
    }

    double safe_delay;
    if (!nh.getParam("/safe_delay", safe_delay)){
        ROS_ERROR("Failed to retrieve safe delay from parameter server.");
        return 1;
    }

    std::vector<std::string> topics;
    if (!nh.getParam("/topics", topics)){
        ROS_ERROR("Failed to retrieve topic names from parameter server.");
        return 1;
    }

    RosbagParser parser(nh, bagpath, safe_delay, topics);
    return 0;
}