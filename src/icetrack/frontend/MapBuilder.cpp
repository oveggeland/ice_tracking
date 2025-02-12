#include "frontend/MapBuilder.h"

MapBuilder::MapBuilder(ros::NodeHandle& nh, const PoseGraph& pose_graph, const FrameBuffer& frame_buffer) 
    : pose_graph_(pose_graph), frame_buffer_(frame_buffer){
    // Optionally load parameters from ros server (using nodehandle)
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map", 10);
    initializeCloudMsg(nh);
}

void MapBuilder::initializeCloudMsg(const ros::NodeHandle& nh){
    // Initialize the PointCloud2 message metadata
    cloud_msg_.header.frame_id = "map";  // Adjust frame ID as needed
    cloud_msg_.height = 1;
    cloud_msg_.is_dense = true;
    cloud_msg_.is_bigendian = false;

    // Define point fields for x, y, z
    cloud_msg_.fields.resize(3);

    cloud_msg_.fields[0].name = "x";
    cloud_msg_.fields[0].offset = 0;
    cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[0].count = 1;

    cloud_msg_.fields[1].name = "y";
    cloud_msg_.fields[1].offset = 4;
    cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[1].count = 1;

    cloud_msg_.fields[2].name = "z";
    cloud_msg_.fields[2].offset = 8;
    cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg_.fields[2].count = 1;

    cloud_msg_.point_step = 12;  // 3 fields * 4 bytes per float
}

void MapBuilder::pollUpdates(){
    // New frames?
    if (frame_buffer_.getLastFrameIdx() > last_frame_idx_)
        updateMap();
}

void MapBuilder::updateMap(){
    last_frame_idx_ = frame_buffer_.getLastFrameIdx();
    ts_map_ = pose_graph_.getTimeStamp(last_frame_idx_);

    int point_count = frame_buffer_.getPointCount();
    ROS_WARN_STREAM("Update map with count " << point_count);
    if (point_count < 1)
        return;

    // Allocate space for transformed points arrays
    map_ = Eigen::MatrixXd(3, point_count);
    int point_counter = 0; // Idx to current column (point)

    // Iterate through frames and transform into point buffer
    for (auto it = frame_buffer_.begin(); it != frame_buffer_.end(); ++it){
        // Check idx and get pose
        int frame_idx = it->frame_idx;
        if (!pose_graph_.exists(frame_idx))
            throw std::out_of_range("updateMap(): Pose " + std::to_string(frame_idx) + " not found");
        
        gtsam::Pose3 pose = pose_graph_.getPose(frame_idx);     
        Eigen::Matrix3d R = pose.rotation().matrix();
        Eigen::Vector3d t = pose.translation();

        // Okay! All good, add transformed points
        int frame_size = it->size();
        map_.block(0, point_counter, 3, frame_size)  = (R * it->positions).colwise() + t;  // Eigen broadcasting!
        point_counter += frame_size;
    }

    publishAsPointCloud2();
}

void MapBuilder::publishAsPointCloud2() {
    if (cloud_pub_.getNumSubscribers() < 1)
        return;

    if (map_.cols() == 0) {
        ROS_WARN_THROTTLE(5, "Map is empty, skipping publish.");
        return;
    }

    // Update message dimensions
    cloud_msg_.width = map_.cols();
    cloud_msg_.row_step = cloud_msg_.point_step * cloud_msg_.width;
    cloud_msg_.data.resize(cloud_msg_.row_step);

    // Fill in the point cloud data using iterators
    sensor_msgs::PointCloud2Iterator<float> it(cloud_msg_, "x");
    for (int i = 0; i < map_.cols(); ++i, ++it) {
        // Set X, Y and Z
        it[0] = map_(0, i);
        it[1] = map_(1, i);
        it[2] = map_(2, i);
    }

    // Update timestamp and publish
    cloud_msg_.header.stamp = ros::Time(ts_map_);
    cloud_pub_.publish(cloud_msg_);
}

/*
Make a open3d cloud and visualize
*/
void MapBuilder::visualizeMap() {
    // Ensure map_ is properly sized and contains points
    int num_points = map_.cols();

    // Create a vector of Eigen::Vector3d to store points
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();
    cloud->points_.reserve(num_points);

    // Convert the Eigen::Matrix3Xd to std::vector<Eigen::Vector3d>
    for (int i = 0; i < num_points; ++i) {
        cloud->points_.push_back(map_.col(i));
    }

    // Normalize
    cloud->Translate(-cloud->GetCenter());
        
    // Visualize the point cloud using Open3D
    open3d::visualization::DrawGeometries({cloud});
}