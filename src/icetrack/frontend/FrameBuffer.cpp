#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer):
                                pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Initialize 
    getParamOrThrow(nh, "frame_buffer/undistort_frames", undistort_frames_);
}

// Check for non-created frames
void FrameBuffer::pollUpdates(){
    int pose_idx = pose_graph_.getCurrentStateIdx();
    int frame_idx = getLastFrameIdx();
    while (frame_idx < pose_idx)
        createFrame(++frame_idx);
}

// Add a frame associated with state "idx"
void FrameBuffer::createFrame(int idx){
    ROS_INFO_STREAM("Add frame: " << idx << " to buffer");

    if (!pose_graph_.exists(idx-1) || !pose_graph_.exists(idx)){
        ROS_WARN_STREAM("Pose not available at idx: " << idx); // If everything goes according to plan, we never end up in this situation?
        return;
    }

    // First access timestamps and poses for current and previous state
    double t0 = pose_graph_.getTimeStamp(idx-1);
    double t1 = pose_graph_.getTimeStamp(idx);

    gtsam::Pose3 pose0 = pose_graph_.getPose(idx-1);
    gtsam::Pose3 pose1 = pose_graph_.getPose(idx);

    // Precompute logmap vectors for interpolation
    gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    gtsam::Vector3 dt_log = b1Tb0.translation() / (t1 - t0); // Velcocity
    gtsam::Vector3 dR_log = gtsam::traits<Rot3>::Logmap(b1Tb0.rotation()) / (t1 - t0); // Angular velocity

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Add new frame to the container
    FrameType& frame = newFrame(idx, num_points);

    // Iterate through points, optionally undistorting by interpolation between poses. 
    for (auto it = start; it != end; ++it) {
        double ts_point = it->ts;
        Eigen::Vector3f position(it->x, it->y, it->z);

        if (undistort_frames_){
            // Interpolate transformation to pose1
            double dt = (t1 - ts_point);
            gtsam::Pose3 T_int( // TODO: Consider first order approximation instead
                gtsam::traits<Rot3>::Expmap(dt*dR_log),
                dt*dt_log
            );
            
            // Transform to pose1
            position = T_int.transformFrom(gtsam::Point3(it->x, it->y, it->z)).cast<float>();
        }

        frame.addPoint(position, it->intensity, ts_point);
    }

    // 
    point_count_ += frame.size();

    // Remove old frames
    removeOldFrames();
}


/*
Add new frame to buffer and return a reference to it.
*/
FrameType& FrameBuffer::newFrame(int idx, size_t capacity){
    buffer_.emplace_back(idx, capacity);
    return buffer_.back();
}

void FrameBuffer::removeOldFrames() {
    // Remove old frames which do not have a corresponding state in pose graph
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (pose_graph_.exists(it->idx())) 
            break;

        point_count_ -= it->size();
        it = buffer_.erase(it);
    }
}

const FrameType& FrameBuffer::getFrame(int idx) const {
    for (const auto& it : buffer_) {
        if (it.idx() == idx) {  // Check for the matching frame_idx
            return it;  // Return the reference to the FrameType object
        }
    }
    
    throw std::out_of_range("Frame not found");  // Handle the case when no matching frame is found
}

bool FrameBuffer::hasFrame(int idx) const {
    for (const auto& it : buffer_) {
        if (it.idx() == idx) {
            return true;  
        }
    }
    return false;
}