#include "frontend/MapBuilder.h"

MapBuilder::MapBuilder(ros::NodeHandle& nh, const PoseGraph& pose_graph, const FrameBuffer& frame_buffer) 
    : pose_graph_(pose_graph), frame_buffer_(frame_buffer){
    // Optionally load parameters from ros server (using nodehandle)
}

void MapBuilder::pollUpdates(){
    // New frames?
    if (frame_buffer_.getLastFrameIdx() > last_frame_idx_)
        updateMap();
}

void MapBuilder::updateMap(){
    last_frame_idx_ = frame_buffer_.getLastFrameIdx();

    int point_count = frame_buffer_.getPointCount();
    if (point_count < 1)
        return;
    ROS_INFO_STREAM("Building map of size: " << point_count);

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
    
    // What now?
    if (point_counter > 1e6)
        visualizeMap();
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