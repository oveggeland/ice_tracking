#include "mapping/CloudManager.h"


CloudManager2::CloudManager2(ros::NodeHandle nh) : cloud_generator_(nh), cloud_processor_(nh), cloud_analyzer_(nh) {
    if (!getParamOrThrow<bool>(nh, "/mapping/enabled")){
        return; // Mapping disabled
    };

    getParamOrThrow(nh, "/mapping/window_size", window_size_);

    // Setup callback sequencer
    double safe_delay = getParamOrThrow<double>(nh, "/mapping/safe_delay");
    sequencer_ = CallbackSequencer(safe_delay);

    // Topic subscribers
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");
    int lidar_queue_size = getParamOrThrow<int>(nh, "/mapping/lidar_queue_size");
    lidar_sub_ = nh.subscribe(lidar_topic, lidar_queue_size, &CloudManager2::lidarCallback, this);

    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    int pose_queue_size = getParamOrThrow<int>(nh, "/mapping/pose_queue_size");
    pose_sub_ = nh.subscribe(pose_topic, pose_queue_size, &CloudManager2::poseCallback, this);
}


// Subscriber callbacks
void CloudManager2::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&CloudManager2::lidarSafeCallback, this, msg));
}

void CloudManager2::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&CloudManager2::poseSafeCallback, this, msg));
}


// Safe callbacks
void CloudManager2::lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // Sequence check
    int seq = msg->header.seq;
    if (lidar_seq_ != 0 && seq - lidar_seq_ != 1)
        ROS_WARN_STREAM("CloudManager2::lidarSafeCallback - Sequence error (" << lidar_seq_ << " vs " << seq << ")");
    lidar_seq_ = seq;

    cloud_generator_.addLidarFrame(msg);
}

void CloudManager2::poseSafeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Sequence check
    int seq = msg->header.seq;
    if (pose_seq_ != 0 && seq - pose_seq_ != 1)
        ROS_WARN_STREAM("CloudManager2::poseSafeCallback - Sequence error (" << pose_seq_ << " vs " << seq << ")");
    pose_seq_ = seq;
    
    cloud_generator_.newPose(msg);
    checkWindowTime(msg->header.stamp.toSec());
}


// Window analysis 
void CloudManager2::checkWindowTime(double ts){
    if (t0_window_ == 0.0)
        t0_window_ = ts;

    else if (ts - t0_window_ > window_size_){
        analyzeWindow(ts);
        
        // Reset
        t0_window_ = ts;
    }
}

void CloudManager2::analyzeWindow(double ts){
    // 1. Generate cloud    
    auto pcd = cloud_generator_.generatePointCloud(t0_window_, ts);
    if (pcd.GetPointPositions().GetShape(0) == 0){
        ROS_WARN("CloudManager2::analyzeWindow - Empty point cloud");
        return;
    }

    // 2. Process cloud
    cloud_processor_.processCloud(ts, pcd);

    // 3. Analyze cloud
    cloud_analyzer_.analyzeCloud(ts, pcd);
}