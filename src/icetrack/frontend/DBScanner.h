#pragma once

#include <queue>

#include <ros/ros.h>

#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/KDTreeFlann.h>

/*
This class implements a version of DBScanner with the option to downsample before the clustering.
The original labels are traced back after clustering is complete. 
*/
class DBScanner{
public:
    // Constructor
    DBScanner(std::shared_ptr<const open3d::geometry::PointCloud> cloud): cloud_(cloud) {
        // First downsample
        downSample();

        // Run DBScan
        ROS_INFO_STREAM("Scan with " << cloud_ds_->points_.size() << " points");
        labels_ds_ = cloud_ds_->ClusterDBSCAN(eps_, min_points_);

        // Rebuild
        traceBack();
    }

    // Use these functions to reduce dimensionality and rebuild original dataset
    void downSample(){
        if (voxel_size_ <= 0){
            ROS_WARN("Can not downsample with zero/negative voxel size");
            return;
        }

        if (cloud_->points_.size() < 1)
            return;

        auto min_bound = cloud_->GetMinBound();
        auto max_bound = cloud_->GetMaxBound();

        std::tie(cloud_ds_, std::ignore, voxel_trace_) = cloud_->VoxelDownSampleAndTrace(voxel_size_, min_bound, max_bound);
    };


    void traceBack(){
        
        ROS_INFO_STREAM("Trace back, number of labels: " << cluster_id_-1);
    }


private:
    double eps_ = 5.0;
    int min_points_ = 50;
    double voxel_size_ = 1.0;

    // Full size stuff
    std::shared_ptr<const open3d::geometry::PointCloud> cloud_;
    std::shared_ptr<open3d::geometry::KDTreeFlann> tree_;
    std::vector<int> labels_;

    // Downsampled stuff
    std::shared_ptr<open3d::geometry::PointCloud> cloud_ds_;
    std::shared_ptr<open3d::geometry::KDTreeFlann> tree_ds_;
    std::vector<std::vector<int>> voxel_trace_;
    std::vector<int> labels_ds_;

    // Results should be 
    // std::vector<int> labels_;
    // std::vector<std::vector<int>> cluster_trace_;
    
    std::vector<int> neighbors_;
    std::vector<double> distances_;
    
    
    int cluster_id_ = 1;
};