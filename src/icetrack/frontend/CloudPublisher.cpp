#include "frontend/CloudPublisher.h"

CloudPublisher::CloudPublisher(ros::NodeHandle& nh, const FrameBuffer& frame_buffer) : frame_buffer_(frame_buffer){
    initializePublisher(nh);
    initializeMessage(nh);
}

void CloudPublisher::initializePublisher(ros::NodeHandle& nh) {
    int pub_queue_size = getParamOrThrow<int>(nh, "/cloud/pub_queue_size");
    std::string pub_topic = getParamOrThrow<std::string>(nh, "/cloud/pub_topic");
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, pub_queue_size);
}

void CloudPublisher::initializeMessage(ros::NodeHandle& nh) {
    cloud_msg_.header.frame_id = "map";  // Publish in nav frame
    cloud_msg_.height = 1;               // Unordered point cloud
    cloud_msg_.is_dense = true;
    cloud_msg_.is_bigendian = false;

    // Define the fields
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
    modifier.setPointCloud2Fields(5, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "i", 1, sensor_msgs::PointField::UINT8,
        "t", 1, sensor_msgs::PointField::FLOAT64
    );
}

void CloudPublisher::fillMessage(){
    // Metadata
    cloud_msg_.header.stamp = ros::Time::now(); // Use latest timestamp
    cloud_msg_.width = frame_buffer_.pointCount();
    cloud_msg_.row_step = cloud_msg_.width*cloud_msg_.point_step;
    cloud_msg_.data.resize(cloud_msg_.row_step);

    // Custom iterator for message
    sensor_msgs::PointCloud2Iterator<PackedPointXYZIT> msg_it(cloud_msg_, "x");

    for (auto frame = frame_buffer_.begin(); frame != frame_buffer_.end(); ++frame){
        auto positions = frame->global();
        auto intensities = frame->intensities();
        auto timestamps = frame->timestamps();

        for (int i = 0; i < frame->size(); ++i, ++msg_it){
            (*msg_it).pos = positions.col(i);
            (*msg_it).i = intensities[i];
            (*msg_it).t = timestamps[i];
        }
    }
}


void CloudPublisher::publishCloud(){
    if (cloud_pub_.getNumSubscribers() == 0)
        return; // No point

    fillMessage();
    if (cloud_msg_.width == 0.0)
        return; // No points

    cloud_pub_.publish(cloud_msg_);
}