#include "LidarFrameBuffer.h"


LidarFrameBuffer::LidarFrameBuffer(const ros::NodeHandle& nh, const LidarPointBuffer& point_buffer) : point_buffer_(point_buffer){
    bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file"));
}

void LidarFrameBuffer::createFrame(int idx, double t0, double t1, const Pose3& pose0, const Pose3& pose1){
    assert(t1 > t0);
    double dt_inv = 1 / (t1 - t0);

    // Find lidar pose relative to pose0
    Pose3 pose0_rel = bTl_;
    Pose3 pose1_rel = pose0.between(pose1).compose(bTl_);

    // Precompute logmap differences between the two poses (for efficient interpolation)
    Vector3 dt_log = pose1_rel.translation() - pose0_rel.translation();
    Vector3 dR_log = Rot3::Logmap(pose0_rel.rotation().between(pose1_rel.rotation()));

    // Find bounds for point buffer iteration
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);
    int num_points = start.distance_to(end);

    // Initialize cloud
    auto cloud = std::make_shared<PointCloud>();
    cloud->points_.reserve(num_points);

    // Transform to pose0_lidar and add to cloud
    for (auto it = start; it != end; ++it){
        // Get stamp
        double ts_point = it->ts;
        assert(ts_point >= t0 && ts_point < t1);

        // Interpolation factor
        double alpha = (ts_point - t0)*dt_inv;
        assert(alpha >= 0 & alpha <= 1);
        
        // Align point (from lidar frame at ts_point to body frame 'pose0')
        Pose3 T_align(
            pose0_rel.rotation().retract(alpha*dR_log),
            pose0_rel.translation() + alpha*dt_log
        );
        gtsam::Point3 point = T_align.transformFrom(Point3(it->x, it->y, it->z));

        // Add to cloud
        cloud->points_.push_back(point); // Raw Lidar points
    }

    frame_buffer_[idx] = cloud;
    frame_buffer_.erase(idx - 10); // Allways delete an old object while we are at it
}

PointCloudSharedPtr LidarFrameBuffer::getFrame(int idx) const{
    auto frame_it = frame_buffer_.find(idx);
    if (frame_it == frame_buffer_.end())
        return nullptr;
    return frame_it->second;
}