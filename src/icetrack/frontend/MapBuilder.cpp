// #include "frontend/MapBuilder.h"

// MapBuilder::MapBuilder(ros::NodeHandle& nh, const PoseGraph& pose_graph, const FrameBuffer& frame_buffer) 
//     : pose_graph_(pose_graph), frame_buffer_(frame_buffer), map_(nh){
// }

// void MapBuilder::pollUpdates(){
//     // New frames?
//     if (frame_buffer_.getLastFrameIdx() > last_frame_idx_)
//         updateMap();
// }

// void MapBuilder::updateMap(){
//     // Reset map with sufficient memory allocation
//     map_.reset(frame_buffer_.getPointCount());

//     // Iterate through frames and transform into point buffer
//     for (auto it = frame_buffer_.begin(); it != frame_buffer_.end(); ++it){
//         // Check idx and get pose
//         int frame_idx = it->frame_idx;
//         if (!pose_graph_.exists(frame_idx))
//             throw std::out_of_range("updateMap(): Pose " + std::to_string(frame_idx) + " not found");
        
//         Eigen::Matrix4f T = pose_graph_.getPose(frame_idx).matrix().cast<float>();     
//         map_.addFrame(T, *it);

//         last_frame_idx_ = frame_idx;
//     }
// }