#pragma once

#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/StampedRingBuffer.h"

// Some useful naming
using PointCloud = open3d::geometry::PointCloud;
using PointCloudSharedPtr = std::shared_ptr<PointCloud>;
using FrameBuffer1 = std::map<double, std::pair<int, PointCloudSharedPtr>>;

using PointBufferType1 = StampedRingBuffer<PointXYZT>;
using PointBufferIterator1 = StampedRingBufferIterator<PointXYZT>;

using namespace gtsam;

// Forward declaration
class PoseGraphManager;

class CloudManager{
public: 
    // Initialize
    CloudManager(ros::NodeHandle& nh);
    void setPoseGraphManager(PoseGraphManager& pose_graph_manager) {pose_graph_manager_ = &pose_graph_manager; }

    // Callback
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        addPoints(msg);
    };

    // Interface (stuff called from Navigation module)
    void newState(int idx){
        createFrame(idx);

        if (ts_cloud_ == 0.0)
            ts_cloud_ = ts_head_;

        if (ts_head_ - ts_cloud_ > cloud_interval_){
            createCloud();
            ts_cloud_ = ts_head_;
        }
    };
    const PointBufferIterator1 pointIteratorLowerBound(double ts) const { return point_buffer_.iteratorLowerBound(ts); }

    PointCloudSharedPtr getFrame(int idx) const{
        for (auto it: frame_buffer_){
            if (it.second.first == idx)
                return it.second.second;
        }   
        return nullptr;
    };


    // Generate a cohorent map, with frames in range (idx0, idx1)
    PointCloudSharedPtr buildMap(int idx0, int idx1) const;

private:
    PoseGraphManager* pose_graph_manager_;

    // Buffers
    PointBufferType1 point_buffer_;
    FrameBuffer1 frame_buffer_;

    // Modify and access buffers
    void addPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void createFrame(int state_idx);

    // Cloud generation
    PointCloudSharedPtr alignFrames(std::vector<Pose3> poses, std::vector<PointCloudSharedPtr> clouds, int point_count);
    void createCloud();

    // Calibration matrix (lidar->imu)
    gtsam::Pose3 bTl_;

    // Cloud management
    double ts_cloud_ = 0.0;
    double cloud_interval_ = 60; 
    double cloud_size_ = 60;

    // Timestamp management
    double ts_head_ = 0.0;
    double point_interval_;

    // Point filtering
    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    // Publisher
    ros::Publisher cloud_pub_;
    void publishCloud(PointCloudSharedPtr pcd) const;

    // Save clouds
    std::string cloud_path_;
    void saveCloud(double ts, const PointCloudSharedPtr pcd) const;
};