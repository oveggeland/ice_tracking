#include "navigation/CloudManager.h"
#include "navigation/PoseGraphManager.h"

CloudManager::CloudManager(ros::NodeHandle& nh){
    // Initialize point buffer
    double buffer_size = getParamOrThrow<double>(nh, "/navigation/surface_estimation/buffer_size");
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_point_interval", point_interval_);
    point_buffer_ = PointBufferType1(buffer_size / point_interval_);

    // Extrinsic calibration
    bTl_ = bTl(getParamOrThrow<std::string>(nh, "/ext_file"));

    // Point filtering configuration
    getParamOrThrow(nh, "/navigation/surface_estimation/lidar_min_intensity", min_intensity_);
    min_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_min_dist"), 2);
    max_dist_squared_ = pow(getParamOrThrow<double>(nh, "/navigation/surface_estimation/lidar_max_dist"), 2);

    // Publisher
    std::string cloud_topic = getParamOrThrow<std::string>(nh, "/cloud_topic");
    int cloud_queue_size = getParamOrThrow<int>(nh, "/cloud_queue_size");
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(cloud_topic, cloud_queue_size);

    // Output
    std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
    cloud_path_ = joinPath(outpath, "clouds/");
    makePath(cloud_path_, true);
}

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


/*
Create a Lidar for state_idx, containing points collected between t0 and t1. 
Interpolation between the poses is used to represent all points in pose1 frame (corresponding to the state_idx pose). 
*/
void CloudManager::createFrame(int state_idx){
    // Retrieve the two latest pose estimates
    auto [t0, pose0] = pose_graph_manager_->getStampedPose(state_idx-1);
    auto [t1, pose1] = pose_graph_manager_->getStampedPose(state_idx);

    // Precompute inverse time delta (needed for interpolation)
    double dt_inv = 1 / (t1 - t0);
    assert (dt_inv > 0);

    // Find transformations to pose1, we will be interpolating between these two transformations. 
    Pose3 T0 = pose1.inverse().compose(pose0).compose(bTl_);         // Transformation from lidar frame at t0 to body frame at t1.
    Pose3 T1 = bTl_;                                                 // Transformation from lidar frame at t1 to body frame at t1. 

    // Precompute logmap differences between the two transformations (for efficient interpolation)
    Vector3 dt_log = T1.translation() - T0.translation();
    Vector3 dR_log = Rot3::Logmap(T0.rotation().between(T1.rotation()));

    // Find bounds for point buffer iteration
    auto start = point_buffer_.iteratorLowerBound(t0);
    auto end = point_buffer_.iteratorLowerBound(t1);
    int num_points = start.distance_to(end);

    // Initialize cloud
    auto cloud = std::make_shared<PointCloud>();
    cloud->points_.reserve(num_points);

    // Iterate through points, interpolate transformation and add to cloud
    for (auto it = start; it != end; ++it){
        // Get stamp
        double ts_point = it->ts;
        assert(ts_point >= t0 && ts_point < t1);

        // Interpolation factor
        double alpha = (ts_point - t0)*dt_inv;
        assert(alpha >= 0 & alpha <= 1);
        
        // Transformation from lidar frame to 'pose1'
        Pose3 T_align(
            T0.rotation().retract(alpha*dR_log),
            T0.translation() + alpha*dt_log
        );
        gtsam::Point3 point = T_align.transformFrom(Point3(it->x, it->y, it->z));

        // Add to cloud
        cloud->points_.push_back(point);
    }

    // Add to frame
    frame_buffer_[t1] = {state_idx, cloud};
    
    auto it = frame_buffer_.lower_bound(t1 - cloud_size_);
    frame_buffer_.erase(frame_buffer_.begin(), it);
}

PointCloudSharedPtr CloudManager::alignFrames(std::vector<Pose3> poses, std::vector<PointCloudSharedPtr> clouds, int point_count){
    assert(poses.size() == clouds.size());

    Point3 offset = poses[0].translation();

    PointCloud pcd_total;
    pcd_total.points_.reserve(point_count);

    for (int i; i < poses.size(); ++i){
        PointCloud cloud(*clouds[i]);

        Pose3 pose = Pose3(
            poses[i].rotation(),
            poses[i].translation() - offset
        );
        cloud.Transform(pose.matrix());

        pcd_total += cloud;
    }

    return std::make_shared<PointCloud>(pcd_total);
}

void CloudManager::createCloud(){
    // Initialize arrays for poses and clouds
    std::vector<Pose3> poses;
    std::vector<PointCloudSharedPtr> clouds;

    int point_count = 0;
    for (auto it: frame_buffer_){
        // Add pose
        auto [ts, pose] = pose_graph_manager_->getStampedPose(it.second.first);
        poses.push_back(pose);

        // Add cloud
        auto cloud = it.second.second;
        clouds.push_back(cloud);

        point_count += cloud->points_.size();
    }

    // Align all frames
    auto pcd = alignFrames(poses, clouds, point_count);

    // Publish?
    publishCloud(pcd);

    // Save cloud in a separate thread
    std::thread(&CloudManager::saveCloud, this, ts_head_, pcd).detach();
}   


void CloudManager::publishCloud(const PointCloudSharedPtr pcd) const{
    if (cloud_pub_.getNumSubscribers() < 1)
        return;
    // Create a ROS PointCloud2 message
    sensor_msgs::PointCloud2 cloud_msg;

    // Set the header for the message
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "world";  // Set your frame here

    // Set the fields for the PointCloud2 message (for XYZ data)
    cloud_msg.fields.resize(3);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;

    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;

    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;

    // Set the point step and row step
    cloud_msg.point_step = 12;  // 3 float32 values (x, y, z), each 4 bytes
    cloud_msg.row_step = cloud_msg.point_step * pcd->points_.size();

    // Resize the data array to hold the point cloud data
    cloud_msg.data.resize(cloud_msg.row_step);

    // Fill the point cloud data
    size_t offset = 0;
    for (const auto& point : pcd->points_) {
        // Copy x, y, z values into the data array
        memcpy(&cloud_msg.data[offset], &point(0), sizeof(float));
        offset += sizeof(float);
        memcpy(&cloud_msg.data[offset], &point(1), sizeof(float));
        offset += sizeof(float);
        memcpy(&cloud_msg.data[offset], &point(2), sizeof(float));
        offset += sizeof(float);
    }

    // Set the width, height, and is_dense (for the PointCloud2 message)
    cloud_msg.width = pcd->points_.size();
    cloud_msg.height = 1;
    cloud_msg.is_dense = true;

    // Publish the PointCloud2 message
    cloud_pub_.publish(cloud_msg);
}

PointCloudSharedPtr CloudManager::buildMap(int idx0, int idx1) const{
    std::vector<Pose3> poses;
    std::vector<PointCloudSharedPtr> clouds;
    std::vector<Point2> drift_vectors;

    int n_frames = idx1 - idx0 + 1;
    poses.reserve(n_frames);
    clouds.reserve(n_frames);
    drift_vectors.reserve(n_frames);

    int total_point_count = 0;
    for (int idx = idx0; idx <= idx1; ++idx){
        auto [ts, pose] = pose_graph_manager_->getStampedPose(idx);
        poses.push_back(pose);
        
        auto cloud = getFrame(idx);
        clouds.push_back(getFrame(idx));
        total_point_count += cloud->points_.size();

        drift_vectors.push_back(pose_graph_manager_->getIceDrift(idx));
    }

    PointCloud map;
    map.points_.reserve(total_point_count);

    for (int i = 0; i < n_frames; ++i){
        Pose3 bTw = Pose3(
            poses[i].rotation(),
            poses[i].translation()// - poses[0].translation()
        );

        // Transform to map
        PointCloud cloud(*clouds[i]);
        cloud.Transform(bTw.matrix());

        // Add to map
        map += cloud;

        // Move entire map
        // Point2 drift = drift_vectors[i];
        // map.Translate(Point3(drift.x(), drift.y(), 0));
    }

    return std::make_shared<PointCloud>(map);
}



void CloudManager::saveCloud(double ts, const PointCloudSharedPtr pcd) const{
    std::stringstream fname;
    fname << std::fixed << static_cast<int>(ts) << ".ply";
    std::string fpath = joinPath(cloud_path_, fname.str());

    // Save the point cloud as a .ply file
    if (open3d::io::WritePointCloud(fpath, *pcd))
        ROS_INFO_STREAM("Saved cloud at " << fpath);
    else
        ROS_WARN_STREAM("Failed to save cloud at " << fpath);
};