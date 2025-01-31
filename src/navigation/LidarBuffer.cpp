#include "LidarBuffer.h"

// Constructor. NodeHandle is required to read configuration from ROS parameters. 
LidarBuffer::LidarBuffer(const ros::NodeHandle& nh) {
    // Initialize point buffer
    double buffer_size = getParamOrThrow<double>(nh, "/navigation/surface_estimation/buffer_size");
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_point_interval", point_interval_);
    point_buffer_ = PointBuffer(buffer_size / point_interval_);

    // Extrinsic calibration
    bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file"));

    // Point filtering configuration
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_max_dist"), 2);
}

/*
Add raw lidar points in point buffer. Outliers are removed. 
*/
void LidarBuffer::addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts_point = msg->header.stamp.toSec() - point_interval_; // Track stamp of points as we iterate
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejections
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
        )
            continue;

        // Range filtering
        double dist_squared = it[0]*it[0] + it[1]*it[1] + it[2]*it[2];
        if (dist_squared < min_dist_squared_ || dist_squared > max_dist_squared_)
            continue;

        // Point accepted
        point_buffer_.addPoint({
            it[0],
            it[1],
            it[2],
            ts_point
        });
    }
    
    if (ts_point > ts_head_){
        ts_head_ = ts_point;
    }
}


/*
Create a LidarFrame, i.e. a cloud containing the set of points from t0 to t1, represented in the frame of pose1 (body/imu frame at t1).
Undistortion is performed by interpolation between the two posese.  
*/
void LidarBuffer::createFrame(int idx, double t0, double t1, const Pose3& pose0, const Pose3& pose1){
    assert(t1 > t0);
    double dt_inv = 1 / (t1 - t0);

    // Find lidar pose relative to pose0 TODO: Fix this shit
    Pose3 pose0_rel = bTl_;
    Pose3 pose1_rel = pose0.between(pose1).compose(bTl_);

    // Precompute logmap differences between the two poses (for efficient interpolation)
    Vector3 dt_log = pose1_rel.translation() - pose0_rel.translation();
    Vector3 dR_log = Rot3::Logmap(pose0_rel.rotation().between(pose1_rel.rotation()));

    // Find bounds for point buffer iteration
    auto start = lowerBoundPointIterator(t0);
    auto end = lowerBoundPointIterator(t1);
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
    frame_buffer_.erase(idx - 10); // TODO: Implement better maintenance haha
}


/*
Request a frame by idx. Return nullptr if not available.
*/
PointCloudSharedPtr LidarBuffer::getFrame(int idx) const{
    auto frame_it = frame_buffer_.find(idx);
    if (frame_it == frame_buffer_.end())
        return nullptr;
    return frame_it->second;
}