#include "frontend/CloudPublisher.h"

CloudPublisher::CloudPublisher(ros::NodeHandle& nh){
    setupRawCloudPublisher(nh);
    setupProcessedCloudPublisher(nh);
}

void CloudPublisher::setupRawCloudPublisher(ros::NodeHandle& nh) {
    // Publisher
    std::string pub_topic = getParamOrThrow<std::string>(nh, "/raw_cloud_topic");
    int pub_queue_size = getParamOrThrow<int>(nh, "/raw_cloud_queue_size_");
    raw_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, pub_queue_size);

    // Message
    raw_cloud_msg_.header.frame_id = "map";  // Publish in nav frame
    raw_cloud_msg_.height = 1;               // Unordered point cloud
    raw_cloud_msg_.is_dense = true;
    raw_cloud_msg_.is_bigendian = false;

    // Define the fields
    sensor_msgs::PointCloud2Modifier modifier(raw_cloud_msg_);
    modifier.setPointCloud2Fields(4, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32
    );
}

void CloudPublisher::setupProcessedCloudPublisher(ros::NodeHandle& nh) {
    // Publisher
    std::string pub_topic = getParamOrThrow<std::string>(nh, "/processed_cloud_topic");
    int pub_queue_size = getParamOrThrow<int>(nh, "/processed_cloud_queue_size_");
    processed_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic, pub_queue_size);

    // Message
    processed_cloud_msg_.header.frame_id = "map";  // Publish in nav frame
    processed_cloud_msg_.height = 1;               // Unordered point cloud
    processed_cloud_msg_.is_dense = true;
    processed_cloud_msg_.is_bigendian = false;

    // Define the fields
    sensor_msgs::PointCloud2Modifier modifier(processed_cloud_msg_);
    modifier.setPointCloud2Fields(5, 
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "deformation", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32
    );
}

void CloudPublisher::publishRawCloud(const std::vector<Eigen::Vector3d>& positions, const std::vector<float>& intensities){
    if (raw_cloud_pub_.getNumSubscribers() == 0)
        return; // No point

    fillRawCloudMessage(positions, intensities);
    if (raw_cloud_msg_.width == 0.0)
        return; // No points

    raw_cloud_pub_.publish(raw_cloud_msg_);
}

void CloudPublisher::publishProcessedCloud(const open3d::t::geometry::PointCloud& cloud){
    if (processed_cloud_pub_.getNumSubscribers() == 0)
        return; // No point

    if (cloud.IsEmpty() || cloud.GetPointPositions().GetShape(0) == 0)
        return; // No points

    fillProcessedCloudMessage(cloud);
    processed_cloud_pub_.publish(processed_cloud_msg_);
}


void CloudPublisher::fillRawCloudMessage(const std::vector<Eigen::Vector3d>& positions, const std::vector<float>& intensities){
    // Metadata
    raw_cloud_msg_.header.stamp = ros::Time::now(); // Use latest timestamp
    raw_cloud_msg_.width = positions.size();
    raw_cloud_msg_.row_step = raw_cloud_msg_.width*raw_cloud_msg_.point_step;
    raw_cloud_msg_.data.resize(raw_cloud_msg_.row_step);

    // Iterate over the cloud and fill message
    sensor_msgs::PointCloud2Iterator<PackedPointXYZI> msg_it(raw_cloud_msg_, "x");
    for (size_t i = 0; i < raw_cloud_msg_.width; ++i, ++msg_it) {
        *msg_it = PackedPointXYZI{
            positions[i].cast<float>(),
            intensities[i]
        };
    }
}




void CloudPublisher::fillProcessedCloudMessage(const open3d::t::geometry::PointCloud& cloud){
    // Metadata
    processed_cloud_msg_.header.stamp = ros::Time::now(); // Use latest timestamp
    processed_cloud_msg_.width = cloud.GetPointPositions().GetShape(0);
    processed_cloud_msg_.row_step = processed_cloud_msg_.width*processed_cloud_msg_.point_step;
    processed_cloud_msg_.data.resize(processed_cloud_msg_.row_step);

    // Get data pointers
    const float* pos_ptr = cloud.GetPointPositions().Contiguous().GetDataPtr<float>();
    
    if (!cloud.HasPointAttr("deformation")){
        ROS_WARN("Cloud has no deformation attribute");
        return;
    }
    const float* deformation_ptr = cloud.GetPointAttr("deformation").Contiguous().GetDataPtr<float>();
    
    if (!cloud.HasPointAttr("intensities")){
        ROS_WARN("Cloud has no intensities attribute");
        return;
    }
    const float* intensity_ptr = cloud.GetPointAttr("intensities").Contiguous().GetDataPtr<float>();
    
    // Iterate over the cloud and fill message
    sensor_msgs::PointCloud2Iterator<PackedPointXYZDI> msg_it(processed_cloud_msg_, "x");
    for (size_t i = 0; i < processed_cloud_msg_.width; ++i) {
        *msg_it = PackedPointXYZDI{
            pos_ptr[0],
            pos_ptr[1],
            pos_ptr[2],
            deformation_ptr[0],
            intensity_ptr[0]
        };
        
        // Increment
        pos_ptr += 3;
        ++deformation_ptr;
        ++intensity_ptr;
        ++msg_it;
    }
}