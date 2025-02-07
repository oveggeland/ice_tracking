#include "mapping/CloudGenerator.h"

CloudGenerator::CloudGenerator(const ros::NodeHandle& nh){
    // Allocate cloud storage
    getParamOrThrow(nh, "/mapping/point_interval", point_interval_);

    double point_buffer_size = getParamOrThrow<double>(nh, "/mapping/point_buffer_size");
    point_buffer_ = StampedRingBuffer<PointXYZIT>(point_buffer_size/point_interval_);

    double cloud_buffer_size = getParamOrThrow<double>(nh, "/mapping/window_size");
    cloud_buffer_ = StampedRingBuffer<PointXYZITDouble>(cloud_buffer_size/point_interval_);

    // Read extrinsics
    bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file"));

    // Config
    getParamOrThrow(nh, "/mapping/shift_cloud", shift_cloud_);
    getParamOrThrow(nh, "/mapping/min_elevation", min_elevation_);
    getParamOrThrow(nh, "/mapping/max_elevation", max_elevation_);
    getParamOrThrow(nh, "/mapping/min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/mapping/min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/mapping/max_dist"), 2);
}


// Add points to lidar buffer
void CloudGenerator::addLidarFrame(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    double ts_point = msg->header.stamp.toSec();

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        if (isValidPoint(it, ts_point)) {
            point_buffer_.addPoint({
                it[0], it[1], it[2], it[3], ts_point
            });
        }

        ts_point += point_interval_; // Points are stamped by a increments from the message header
    }

    ts_head_ = std::max(ts_head_, ts_point); // Update timestamp if needed
}

bool CloudGenerator::isValidPoint(const sensor_msgs::PointCloud2ConstIterator<float>& it, double ts_point) const{
    return (ts_point > ts_head_ &&                  // Valid timestamp
            it[0] != 0.0 &&                         // Not an empty point
            it[3] >= min_intensity_ &&              // Above intensity threshold
            isWithinRange(it[0], it[1], it[2])      // Within range requirements
    );
}

bool CloudGenerator::isWithinRange(float x, float y, float z) const{
    double dist_squared = x*x + y*y + z*z;
    return (dist_squared >= min_dist_squared_ && dist_squared <= max_dist_squared_);
}
    

// New pose, interpolation and add to cloud buffer
void CloudGenerator::newPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Extract msg data
    double t_pose = msg->header.stamp.toSec();
    gtsam::Pose3 pose_imu = poseRosToGtsam(msg->pose);
    gtsam::Pose3 pose_lidar = pose_imu.compose(bTl_);

    // Initialize and shift pose if necessary
    if (t_pose_prev_ == 0.0){
        initialize(t_pose, pose_lidar);
        return;
    }
    if (shift_cloud_)
        pose_lidar = shiftPose(pose_lidar);

    // Precompute logmap differences between the two poses (for efficient interpolation)
    gtsam::Vector3 dt_log = pose_lidar.translation() - pose_prev_.translation();
    gtsam::Vector3 dR_log = gtsam::Rot3::Logmap(pose_prev_.rotation().between(pose_lidar.rotation()));

    double dt_inv = 1.0 / (t_pose - t_pose_prev_);
    assert(dt_inv > 0); 

    // Find bounds for point buffer iteration
    auto start = point_buffer_.iteratorLowerBound(t_pose_prev_);
    auto end = point_buffer_.iteratorLowerBound(t_pose);

    // Iterate over all points
    for (auto it = start; it != end; ++it){
        double ts_point = it->ts;
        assert(ts_point >= t0_ && ts_point < t1);

        // Pose interpolation
        double alpha = (ts_point - t_pose_prev_)*dt_inv;
        assert(alpha >= 0 & alpha <= 1);
        gtsam::Pose3 wTl(
            pose_prev_.rotation().retract(alpha*dR_log),
            pose_prev_.translation() + alpha*dt_log
        );
        assert(wTl.equals(pose_prev_.interpolateRt(pose_lidar, alpha))); // I know interpolateRt works, but my way is more efficient. 

        // Transform to world frame
        gtsam::Point3 wPwp = wTl.transformFrom(gtsam::Point3(it->x, it->y, it->z));

        // Filter outliers on elevation
        double elevation = -wPwp.z();
        if (elevation < min_elevation_ || elevation > max_elevation_)
            continue; 

        // Add point!
        cloud_buffer_.addPoint({
            wPwp.x(), 
            wPwp.y(),
            wPwp.z(),
            it->i,
            ts_point
        });
    }

    t_pose_prev_ = t_pose;
    pose_prev_ = pose_lidar;
}

void CloudGenerator::initialize(double t0, gtsam::Pose3 pose0){
    t_pose_prev_ = t0;

    if (shift_cloud_){
        x_shift_ = pose0.translation().x();
        y_shift_ = pose0.translation().y();
        pose_prev_ = shiftPose(pose0);
    }
    else{
        pose_prev_ = pose0;
    }
}

gtsam::Pose3 CloudGenerator::shiftPose(const gtsam::Pose3& pose) const{
    return gtsam::Pose3(pose.rotation(), pose.translation() - gtsam::Point3(x_shift_, y_shift_, 0));
}

// Generate a cloud with points from time interval (t0, t1)
open3d::t::geometry::PointCloud CloudGenerator::generatePointCloud(double t0, double t1) const{
    // Find bounds for point buffer iteration
    auto start = cloud_buffer_.iteratorLowerBound(t0);
    auto end = cloud_buffer_.iteratorLowerBound(t1);

    // Allocate memory
    int num_points = start.distance_to(end);
    ROS_WARN_STREAM("Generating cloud of size: " << num_points);
    
    std::vector<float> positions;
    positions.reserve(3*num_points);

    std::vector<float> intensities;
    intensities.reserve(num_points);

    // Add points to vectors
    for (auto it = start; it != end; ++it){
        positions.push_back(it->x);
        positions.push_back(it->y);
        positions.push_back(it->z);
        intensities.push_back(it->i);
    }

    // Generate pointcloud and set attributes
    auto pcd = open3d::t::geometry::PointCloud();
    pcd.SetPointPositions(open3d::core::Tensor(std::move(positions), {num_points, 3}, open3d::core::Dtype::Float32));
    pcd.SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities), {num_points, 1}));

    return pcd;
}