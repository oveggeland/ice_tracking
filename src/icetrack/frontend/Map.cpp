// #include "frontend/MapBuilder.h"

// Map::Map(ros::NodeHandle& nh) {
//     cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map", 10);
//     initializeCloudMsg();
// }


// void Map::reset(size_t size){
//     positions_.resize(3, size);
//     intensities_.resize(1, size);
//     timestamps_.resize(1, size);
//     size_ = 0;
// };


// void Map::addFrame(const Eigen::Matrix4f& T, const FrameType& frame){
//     size_t new_size = size() + frame.size();
//     if (new_size > capacity())
//         ROS_ERROR("addFrame - Not enough capacity");

//     const Eigen::Matrix3f& R = T.block<3, 3>(0, 0);
//     const Eigen::Vector3f& t = T.block<3, 1>(0, 3);
//     positions_.block(0, size(), 3, frame.size()) = (R * frame.positions).colwise() + t;

//     intensities_.block(0, size(), 1, frame.size()) = frame.intensities;
//     timestamps_.block(0, size(), 1, frame.size()) = frame.timestamps;

//     size_ += frame.size();
// }


// void Map::initializeCloudMsg(){
//     // Initialize the PointCloud2 message metadata
//     cloud_msg_.header.frame_id = "map";  // Adjust frame ID as needed
//     cloud_msg_.height = 1;
//     cloud_msg_.is_dense = true;
//     cloud_msg_.is_bigendian = false;

//     // Define point fields for x, y, z, i
//     cloud_msg_.fields.resize(4);

//     cloud_msg_.fields[0].name = "x";
//     cloud_msg_.fields[0].offset = 0;
//     cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
//     cloud_msg_.fields[0].count = 1;

//     cloud_msg_.fields[1].name = "y";
//     cloud_msg_.fields[1].offset = 4;
//     cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
//     cloud_msg_.fields[1].count = 1;

//     cloud_msg_.fields[2].name = "z";
//     cloud_msg_.fields[2].offset = 8;
//     cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
//     cloud_msg_.fields[2].count = 1;

//     cloud_msg_.fields[3].name = "i";
//     cloud_msg_.fields[3].offset = 12;
//     cloud_msg_.fields[3].datatype = sensor_msgs::PointField::UINT8;
//     cloud_msg_.fields[3].count = 1;


//     cloud_msg_.point_step = 13;  // 3 fields * 4 bytes per float
// }


// /*
// Publish map on ROS topic.
// */
// void Map::publishAsPointCloud2(){
//     if (cloud_pub_.getNumSubscribers() < 1)
//         return;

//     if (size() == 0) {
//         ROS_WARN_THROTTLE(5, "Map is empty, skipping publish.");
//         return;
//     }

//     // Update message dimensions
//     cloud_msg_.width = size();
//     cloud_msg_.row_step = cloud_msg_.point_step * cloud_msg_.width;
//     cloud_msg_.data.resize(cloud_msg_.row_step);

//     // Fill in the point cloud data using iterators
//     sensor_msgs::PointCloud2Iterator<float> it(cloud_msg_, "x");
//     sensor_msgs::PointCloud2Iterator<uint8_t> it_intensity(cloud_msg_, "i");

//     for (int i = 0; i < size(); ++i, ++it, ++it_intensity) {
//         // Set X, Y and Z
//         it[0] = positions_(0, i);
//         it[1] = positions_(1, i);
//         it[2] = positions_(2, i);

//         // Set Intensity
//         *it_intensity = intensities_(i);
//     }

//     // Update timestamp and publish
//     cloud_msg_.header.stamp = ros::Time::now();
//     cloud_pub_.publish(cloud_msg_);
// }

// /*
// Make a open3d cloud and visualize
// */
// void Map::visualizeMap() const{
//     auto cloud_ptr = EigenToPointCloudPtr(positions_);
//     open3d::visualization::DrawGeometries({cloud_ptr});
// }