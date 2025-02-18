#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer):
                                pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Initialize 
    getParamOrThrow(nh, "frame_buffer/undistort_frames", undistort_frames_);
}


// Add a frame associated with state "idx"
bool FrameBuffer::createFrame(int idx){
    double t0, t1;
    gtsam::Pose3 pose0, pose1;
    
    if (!pose_graph_.timePoseQuery(idx-1, t0, pose0) || !pose_graph_.timePoseQuery(idx, t1, pose1)){
        ROS_WARN_STREAM("Pose not available at idx: " << idx); // If everything goes according to plan, we never end up in this situation?
        return false;
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
    buffer_.emplace_back(idx, num_points);
    FrameType& frame = buffer_.back();

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
    
    frame.transformPoints(pose1.matrix().cast<float>());

    point_count_ += frame.size();
    return true;
}


/*
Iterate over frames and remove frames that are too old or if the pose graph state is not available.
*/
void FrameBuffer::removeOldFrames() {
    double ts_threshold = ros::Time::now().toSec() - window_size_;
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (it->t1() > ts_threshold && pose_graph_.exists(it->idx()))
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


/*
Return shared ptr with a cloud frame containing all points. Attributes are decided by input parameters.
*/
CloudFrame::Ptr FrameBuffer::getPoints(bool local, bool global, bool intensities, bool timestamps) const {
    // Allocate CloudFrame with capacity
    CloudFrame::Ptr frame = std::make_shared<CloudFrame>(0, point_count_);

    // Iterate through all frames and merge points
    for (const auto& it : buffer_)
        frame->merge(it, local, global, intensities, timestamps);
    return frame;
}


/*
Return shared ptr with a cloud frame containing all points. Attributes are decided by input parameters.
*/
CloudFrame::Ptr FrameBuffer::getPoints(double t0, double t1, bool local, bool global, bool intensities, bool timestamps) const {
    std::vector<CloudFrame> blocks;
    blocks.reserve(size());

    // Iterate through and fetch blocks
    int point_count = 0;
    for (const auto& it : buffer_) {
        if (it.t0() < t1 && it.t1() > t0) {
            blocks.push_back(it.block(t0, t1));
            point_count += blocks.back().size();
        }
    }

    // Allocate CloudFrame with sufficient capacity
    CloudFrame::Ptr frame = std::make_shared<CloudFrame>(0, point_count);

    // Merge all blocks
    for (const auto& block : blocks) 
        frame->merge(block, local, global, intensities, timestamps);

    return frame;
}