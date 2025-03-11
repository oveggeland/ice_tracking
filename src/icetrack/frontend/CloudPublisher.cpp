#include "frontend/CloudPublisher.h"

CloudPublisher::CloudPublisher(ros::NodeHandle& nh){
    setupPublishers(nh);
    initializeMessages();
}

void CloudPublisher::setupPublishers(ros::NodeHandle& nh) {
    // Frame pub
    std::string frame_topic = getParamOrThrow<std::string>(nh, "/cloud_publisher/frame_topic");
    frame_pub_ = nh.advertise<sensor_msgs::PointCloud2>(frame_topic, 10);

    // Cloud pub
    std::string cloud_topic = getParamOrThrow<std::string>(nh, "/cloud_publisher/cloud_topic");
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, 10);
}

void CloudPublisher::initializeMessages(){
    // Common values
    sensor_msgs::PointCloud2 msg_common;
    msg_common.height = 1;
    msg_common.is_dense = true;
    msg_common.is_bigendian = false;

    // Define the fields
    sensor_msgs::PointCloud2Modifier modifier(msg_common);
    modifier.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32
    );

    // Specify frame id's for frame and cloud
    frame_msg_ = msg_common;
    frame_msg_.header.frame_id = "body";  // Publish in body frame

    cloud_msg_ = msg_common;
    cloud_msg_.header.frame_id = "map";  // Publish in nav frame
}

void CloudPublisher::publishCloud(const std::vector<PointXYZI>& points){
    if (cloud_pub_.getNumSubscribers() == 0)
        return; // No point

    fillCloudMessage(cloud_msg_, points);
    if (cloud_msg_.width == 0.0)
        return; // No points

    cloud_pub_.publish(cloud_msg_);
}

void CloudPublisher::publishFrame(const std::vector<Eigen::Vector3d>& positions, const std::vector<float>& intensities){
    if (frame_pub_.getNumSubscribers() == 0)
        return; // No point

    fillCloudMessage(frame_msg_, positions, intensities);
    if (frame_msg_.width == 0.0)
        return; // No points

    frame_pub_.publish(frame_msg_);
}

void CloudPublisher::fillCloudMessage(sensor_msgs::PointCloud2& msg, 
                                        const std::vector<PointXYZI>& points){
    // Metadata
    msg.header.stamp = ros::Time::now();
    msg.width = points.size();
    msg.row_step = msg.width*msg.point_step;
    msg.data.resize(msg.row_step);

    // Memcpy from points to msg.data
    std::memcpy(msg.data.data(), points.data(), msg.data.size());
}

void CloudPublisher::fillCloudMessage(sensor_msgs::PointCloud2& msg, 
                                        const std::vector<Eigen::Vector3d>& positions, 
                                        const std::vector<float>& intensities){
    // Metadata
    msg.header.stamp = ros::Time::now();
    msg.width = positions.size();
    msg.row_step = msg.width*msg.point_step;
    msg.data.resize(msg.row_step);

    // Iterate over the cloud and fill message
    sensor_msgs::PointCloud2Iterator<PackedPointXYZI> msg_it(msg, "x");
    for (size_t i = 0; i < msg.width; ++i, ++msg_it) {
        *msg_it = PackedPointXYZI{
            positions[i].cast<float>(),
            intensities[i]
        };
    }
}