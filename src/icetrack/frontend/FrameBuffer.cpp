#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer):
                                pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Initialize 
    getParamOrThrow(nh, "frame_buffer/undistort_frames", undistort_frames_);
}

// Check for non-created frames
void FrameBuffer::generateFrames(){
    int last_pose_idx = pose_graph_.getCurrentStateIdx();
    int last_frame_idx = buffer_.empty() ? 0 : buffer_.rbegin()->first;
    while (last_frame_idx < last_pose_idx)
        addFrame(++last_frame_idx);
}

// Add a frame associated with state "idx"
void FrameBuffer::addFrame(int idx){
    ROS_DEBUG_STREAM("Add frame: " << idx << " to buffer");

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
    Pose3 b1Tb0 = pose1.between(pose0);
    Vector3 dt_log = b1Tb0.translation();
    Vector3 dR_log = Rot3::Logmap(b1Tb0.rotation());

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Allocate memory for position and intensity
    std::vector<double> points;
    std::vector<float> intensities;
    points.reserve(3*num_points);
    intensities.reserve(num_points);

    // Iterate through points, optionally undistorting by interpolation between poses. 
    for (auto it = start; it != end; ++it) {
        gtsam::Point3 point = Point3(it->x, it->y, it->z);

        if (undistort_frames_){
            // Get stamp
            double ts_point = it->ts;
            assert(ts_point >= t0 && ts_point < t1);

            // Interpolate transformation to pose1
            double alpha = (t1 - ts_point) * dt_inv;
            Pose3 T_int(
                Rot3::Expmap(alpha*dR_log),
                alpha*dt_log
            );
            
            // Transform to pose1
            point = T_int.transformFrom(point);
        }

        // Store transformed point and intensity
        points.push_back(point.x());
        points.push_back(point.y());
        points.push_back(point.z());
        intensities.push_back(it->intensity);
    }

    // Convert to Open3D tensor cloud
    auto cloud = std::make_shared<TensorCloud>();
    cloud->SetPointPositions(open3d::core::Tensor(std::move(points), {num_points, 3}, open3d::core::Float64, open3d::core::Device("CPU:0")));
    cloud->SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities), {num_points, 1}, open3d::core::Float32, open3d::core::Device("CPU:0")));

    // Add to buffer
    buffer_[idx] = cloud;

    // Remove old frames
    removeOldFrames();
}

void FrameBuffer::removeOldFrames() {
    // Remove old frames which do not have a corresponding state in pose graph
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (pose_graph_.exists(it->first)) 
            break;

        it = buffer_.erase(it);
    }
}
