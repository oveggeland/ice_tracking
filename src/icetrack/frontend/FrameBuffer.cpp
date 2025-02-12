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

    // Precompute inverse time delta (needed for interpolation)
    double dt_inv = 1 / (t1 - t0);
    assert (dt_inv > 0);

    // Precompute logmap vectors for relative transformation from pose0 to pose1. 
    gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    gtsam::Vector3 dt_log = b1Tb0.translation();
    gtsam::Vector3 dR_log = gtsam::traits<Rot3>::Logmap(b1Tb0.rotation());

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Add new frame to the container
    FrameType& frame = newFrame(num_points);
    frame.frame_idx = idx;
    int point_counter = 0;

    // Iterate through points, optionally undistorting by interpolation between poses. 
    for (auto it = start; it != end; ++it) {
        gtsam::Point3 point(it->x, it->y, it->z);

        if (undistort_frames_){
            // Get stamp
            double ts_point = it->ts;
            assert(ts_point >= t0 && ts_point < t1);

            // Interpolate transformation to pose1
            double alpha = (t1 - ts_point) * dt_inv;
            gtsam::Pose3 T_int(
                gtsam::traits<Rot3>::Expmap(alpha*dR_log),
                alpha*dt_log
            );
            
            // Transform to pose1
            point = T_int.transformFrom(point);
        }

        frame.positions.col(point_counter) = point.cast<float>();
        frame.intensities(point_counter) = it->intensity;
        ++ point_counter;
    }

    // 
    point_count_ += frame.size();

    // Remove old frames
    removeOldFrames();
}


FrameType& FrameBuffer::newFrame(int num_points){
    buffer_.emplace_back(num_points);  // Add a new frame (initialized with num_points)
    return buffer_.back();  // Return reference to the newly added frame
}


void FrameBuffer::removeOldFrames() {
    // Remove old frames which do not have a corresponding state in pose graph
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (pose_graph_.exists(it->frame_idx)) 
            break;

        point_count_ -= it->size();
        it = buffer_.erase(it);
    }
}

const FrameType& FrameBuffer::getFrame(int idx) const {
    for (const auto& it : buffer_) {  // Iterate by reference to avoid copying
        if (it.frame_idx == idx) {  // Check for the matching frame_idx
            return it;  // Return the reference to the FrameType object
        }
    }
    
    throw std::out_of_range("Frame not found");  // Handle the case when no matching frame is found
}

bool FrameBuffer::hasFrame(int idx) const {
    for (const auto& it : buffer_) {
        if (it.frame_idx == idx) {
            return true;  
        }
    }
    return false;
}