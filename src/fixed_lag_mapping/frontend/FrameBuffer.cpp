#include "FrameBuffer.h"

/*
Constructor reads config from parameter server.
*/
FrameBuffer::FrameBuffer(const ros::NodeHandle& nh){
    getParamOrThrow(nh, "/frame_buffer/window_size", window_size_);
}

/*
Emplace a frame, and return a reference to it. 
*/
LidarFrame& FrameBuffer::emplaceFrame(const int frame_id, const double ts, const size_t capacity) {
    num_points_ += capacity;
    return buffer_.emplace_back(frame_id, ts, capacity);
}


/*
Maintain buffer by removing out-of-scope frames
*/
void FrameBuffer::maintain(){
    const double ts_threshold = ros::Time::now().toSec() - window_size_;
    while (!buffer_.empty() && buffer_.front().timestamp() < ts_threshold) {
        num_points_ -= buffer_.front().size();  // Update point count
        buffer_.pop_front();
    }
}


/*
Query a frame by index. Return nullptr if frame is non-existent.
*/
LidarFrame* FrameBuffer::getFrame(const int frame_id) {
    for (auto& it : buffer_) {
        if (it.id() == frame_id) {
            return &it;  // Return a pointer to the found frame
        }
    }
    return nullptr;  // Frame not found
}

/*
Const version of above function. 
*/
const LidarFrame* FrameBuffer::getFrame(const int frame_id) const{
    for (auto& it : buffer_) {
        if (it.id() == frame_id) {
            return &it;  // Return a pointer to the found frame
        }
    }
    return nullptr;  // Frame not found
}