#include "navigation/CloudManager.h"
#include "navigation/PoseGraphManager.h"

CloudManager::CloudManager(ros::NodeHandle& nh){
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

void CloudManager::setPoseGraphManager(PoseGraphManager& pose_graph_manager) { pose_graph_manager_ = &pose_graph_manager; }

/*
Add raw lidar points in point buffer. Outliers are removed. 
*/
void CloudManager::addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg){
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


// /*
// Create a Lidar for state_idx, containing points collected between t0 and t1. 
// Interpolation between the poses is used to represent all points in pose1 frame (corresponding to the state_idx pose). 
// */
// void LidarBuffer::createFrame(int state_idx, double t0, double t1, const Pose3& pose0, const Pose3& pose1){
//     // Precompute inverse time delta (needed for interpolation)
//     double dt_inv = 1 / (t1 - t0);
//     assert (dt_inv > 0);

//     // Find transformations to pose1, we will be interpolating between these two transformations. 
//     Pose3 T0 = pose1.inverse().compose(pose0).compose(bTl_);         // Transformation from lidar frame at t0 to body frame at t1.
//     Pose3 T1 = bTl_;                                                 // Transformation from lidar frame at t1 to body frame at t1. 

//     // Precompute logmap differences between the two transformations (for efficient interpolation)
//     Vector3 dt_log = T1.translation() - T0.translation();
//     Vector3 dR_log = Rot3::Logmap(T0.rotation().between(T1.rotation()));

//     // Find bounds for point buffer iteration
//     auto start = lowerBoundPointIterator(t0);
//     auto end = lowerBoundPointIterator(t1);
//     int num_points = start.distance_to(end);

//     // Initialize cloud
//     auto cloud = std::make_shared<PointCloud>();
//     cloud->points_.reserve(num_points);

//     // Iterate through points, interpolate transformation and add to cloud
//     for (auto it = start; it != end; ++it){
//         // Get stamp
//         double ts_point = it->ts;
//         assert(ts_point >= t0 && ts_point < t1);

//         // Interpolation factor
//         double alpha = (ts_point - t0)*dt_inv;
//         assert(alpha >= 0 & alpha <= 1);
        
//         // Transformation from lidar frame to 'pose1'
//         Pose3 T_align(
//             T0.rotation().retract(alpha*dR_log),
//             T0.translation() + alpha*dt_log
//         );
//         gtsam::Point3 point = T_align.transformFrom(Point3(it->x, it->y, it->z));

//         // Add to cloud
//         cloud->points_.push_back(point);
//     }

//     // Add to frame
//     frame_buffer_[state_idx] = cloud;
//     frame_buffer_.erase(state_idx - 10); // TODO: Implement better maintenance haha
// }
