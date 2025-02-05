#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(const ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer):
                                pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Initialize 
}

// Check for non-created frames
void FrameBuffer::generateFrames(){
    int idx = pose_graph_.getCurrentStateIdx();
    while (frame_count_ < idx)
        addFrame(++frame_count_);
}


void FrameBuffer::addFrame(int idx){
    if (!pose_graph_.exists(idx-1) || !pose_graph_.exists(idx)){
        ROS_WARN_STREAM("Pose not available at idx: " << idx);
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

    // Find transformations to pose1, we will be interpolating between these two transformations. 
    gtsam::Pose3 T0 = pose1.between(pose0);
    gtsam::Pose3 T1 = gtsam::Pose3();

    // Precompute logmap differences between the two transformations (for efficient interpolation)
    gtsam::Vector3 dt_log = T1.translation() - T0.translation();
    gtsam::Vector3 dR_log = Rot3::Logmap(T0.rotation().between(T1.rotation()));

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Initialize cloud
    auto cloud = std::make_shared<TensorCloud>();

    // Prepare storage for points and intensity (Nx3 for XYZ, Nx1 for intensity)
    std::vector<double> points;
    std::vector<float> intensities;
    points.reserve(3*num_points);
    intensities.reserve(num_points);

    // Iterate through points, interpolate transformation and add to cloud
    for (auto it = start; it != end; ++it) {
        // Get stamp
        double ts_point = it->ts;
        assert(ts_point >= t0 && ts_point < t1);

        // Interpolation factor
        double alpha = (ts_point - t0) * dt_inv;
        assert(alpha >= 0 && alpha <= 1);

        // Transformation from lidar frame to 'pose1'
        Pose3 T_align(
            T0.rotation().retract(alpha*dR_log),
            T0.translation() + alpha*dt_log
        );
        gtsam::Point3 point = T_align.transformFrom(Point3(it->x, it->y, it->z));

        // Store transformed point and intensity
        points.push_back(point.x());
        points.push_back(point.y());
        points.push_back(point.z());
        intensities.push_back(it->intensity);
    }

    // Convert to Open3D tensors
    cloud->SetPointPositions(open3d::core::Tensor(points, {num_points, 3}, open3d::core::Float64, open3d::core::Device("CPU:0")));
    cloud->SetPointAttr("intensities", open3d::core::Tensor(intensities, {num_points, 1}, open3d::core::Float32, open3d::core::Device("CPU:0")));

    // Add to frame
    ROS_INFO_STREAM("Created frame number: " << idx);
    buffer_[idx] = cloud;

    // TODO: Maintain
}