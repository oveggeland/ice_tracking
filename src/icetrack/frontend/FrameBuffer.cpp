#include "FrameBuffer.h"

FrameBuffer::FrameBuffer(ros::NodeHandle& nh, const PoseGraph& pose_graph, const PointBuffer& point_buffer):
                                pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Initialize 
    getParamOrThrow(nh, "frame_buffer/undistort_frames", undistort_frames_);
    initializePublisher(nh);
}

void FrameBuffer::initializePublisher(ros::NodeHandle& nh) {
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_topic", 10);

    cloud_msg_.header.frame_id = "map";  // Publish in nav frame
    cloud_msg_.height = 1;               // Unordered point cloud
    cloud_msg_.is_dense = true;
    cloud_msg_.is_bigendian = false;

    // Define point fields: x, y, z (float32), intensity (uint8_t), timestamp (float64)
    cloud_msg_.fields.resize(5);
    int offset = 0;

    cloud_msg_.fields[0].name = "x";
    cloud_msg_.fields[0].offset = offset;
    cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    offset += 4;

    cloud_msg_.fields[1].name = "y";
    cloud_msg_.fields[1].offset = offset;
    cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    offset += 4;

    cloud_msg_.fields[2].name = "z";
    cloud_msg_.fields[2].offset = offset;
    cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    offset += 4;

    cloud_msg_.fields[3].name = "i";
    cloud_msg_.fields[3].offset = offset;
    cloud_msg_.fields[3].datatype = sensor_msgs::PointField::UINT8;
    offset += 1;

    cloud_msg_.fields[4].name = "t";
    cloud_msg_.fields[4].offset = offset;
    cloud_msg_.fields[4].datatype = sensor_msgs::PointField::FLOAT64;
    offset += 8;

    cloud_msg_.point_step = offset; 
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
    cloud_size_ += frame.size();
}


/*
Add new frame to buffer and return a reference to it.
*/
FrameType& FrameBuffer::addFrame(int idx, size_t capacity){
    buffer_.emplace_back(idx, capacity);
    return buffer_.back();
}


/*
Iterate over frames and remove frames that are too old or if the pose graph state is not available.
*/
void FrameBuffer::removeOldFrames() {
    double ts_threshold = ros::Time::now().toSec() - window_size_;
    for (auto it = buffer_.begin(); it != buffer_.end(); ) {
        if (it->t1() > ts_threshold && pose_graph_.exists(it->idx()))
            break;
        cloud_size_ -= it->size();
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
Merge all points within the interval.
*/
Eigen::Matrix3Xf FrameBuffer::getPointsWithin(double t0, double t1) const{
    size_t num_points = 0;
    std::vector<Eigen::Matrix3Xf> blocks;
    blocks.reserve(size());

    // Iterate through buffer and get view to all valid point blocks
    for (const auto& it: buffer_){
        Eigen::Matrix3Xf block = it.getPointsWithin(t0, t1);

        if (block.cols() > 0){
            blocks.push_back(block);
            num_points += block.cols();
        }
    }

    // Now merge all blocks
    Eigen::Matrix3Xf points(3, num_points);
    int p_cnt = 0;
    for (const auto& block : blocks) {
        points.block(0, p_cnt, 3, block.cols()) = block;
        p_cnt += block.cols();
    }
    
    return points;
}

/*
Publish pointcloud 2 with x,y,z (float), intensity (uint8_t) and timestamp (double)
*/
void FrameBuffer::publishCloud() {
    if (cloud_pub_.getNumSubscribers() == 0 || cloud_size_ == 0) return;

    // msg.header.stamp = ros::Time::now(); // Use latest timestamp
    // msg.row_step = msg.point_step*cloud_size_;
    // msg.width = cloud_size_;


    // sensor_msgs::PointCloud2Iterator<float> it_pos(msg, "x");
    // sensor_msgs::PointCloud2Iterator<uint8_t> it_i(msg, "i");
    // sensor_msgs::PointCloud2Iterator<double> it_t(msg, "t");

    // for (size_t i = 0; i < cloud_.size(); ++i, ++it_pos, ++it_i, ++it_t) {
    //     *it_pos[0] = cloud_[i].x();
    //     *iter_y = cloud_[i].y();
    //     *iter_z = cloud_[i].z();
    //     *iter_intensity = static_cast<uint8_t>(cloud_[i].intensity);
    //     *iter_timestamp = cloud_[i].timestamp;
    // }

    // cloud_pub_.publish(msg); // Assuming you have a ROS publisher `cloud_pub_`
}