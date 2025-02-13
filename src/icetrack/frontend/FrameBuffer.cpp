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

    if (pose_idx == frame_idx)
        return;

    // Add new frames
    while (frame_idx < pose_idx)
        createFrame(++frame_idx);

    // Delete old ones
    removeOldFrames();

    // Update frames
    refineFrames();
}

// Add a frame associated with state "idx"
void FrameBuffer::createFrame(int idx){
    ROS_INFO_STREAM("Add frame: " << idx << " to buffer");

    double t0, t1;
    gtsam::Pose3 pose0, pose1;
    
    if (!pose_graph_.timePoseQuery(idx-1, t0, pose0) || !pose_graph_.timePoseQuery(idx, t1, pose1)){
        ROS_WARN_STREAM("Pose not available at idx: " << idx); // If everything goes according to plan, we never end up in this situation?
        return;
    }

    // Precompute logmap vectors for interpolation
    gtsam::Pose3 b1Tb0 = pose1.between(pose0);
    gtsam::Vector3 dt_log = b1Tb0.translation() / (t1 - t0); // Velcocity
    gtsam::Vector3 dR_log = gtsam::traits<Rot3>::Logmap(b1Tb0.rotation()) / (t1 - t0); // Angular velocity

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Add new frame to the container
    FrameType& frame = addFrame(idx, num_points);

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
}


/*
Add new frame to buffer and return a reference to it.
*/
FrameType& FrameBuffer::addFrame(int idx, size_t capacity){
    buffer_.emplace_back(idx, capacity);
    return buffer_.back();
}


/*
Iterate over frames and remove frames where pose graph state is not available.
*/
void FrameBuffer::removeOldFrames() {
    // Remove old frames which do not have a corresponding state in pose graph
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (pose_graph_.exists(it->idx())) 
            break;

        point_count_ -= it->size();
        it = buffer_.erase(it);
    }
}

/*
Update global position with refined pose.
*/
void FrameBuffer::refineFrames() {
    for (auto it = buffer_.begin(); it != buffer_.end(); ++it) {
        gtsam::Pose3 pose = pose_graph_.getPose(it->idx());
        it->transformPoints(pose.matrix().cast<float>());
    }
}


/*
Query a frame by index. Return nullptr if frame is non-existent.
*/
const FrameType* FrameBuffer::getFrame(int idx) const {
    for (const auto& it : buffer_) {
        if (it.idx() == idx) {
            return &it;  // Return a pointer to the found frame
        }
    }
    return nullptr;  // Frame not found
}