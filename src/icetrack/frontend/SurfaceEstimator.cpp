#include "SurfaceEstimator.h"

SurfaceEstimator::SurfaceEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const PointBuffer& point_buffer) : pose_graph_(pose_graph), point_buffer_(point_buffer) {
    // Get config from ros params
    getParamOrThrow(nh, "/surface_estimator/window_size", window_size_);
    getParamOrThrow(nh, "/surface_estimator/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/surface_estimator/min_inlier_count", min_inlier_count_);
    
    getParamOrThrow(nh, "/surface_estimator/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh, "/surface_estimator/ransac_sample_size", ransac_sample_size_);
    getParamOrThrow(nh, "/surface_estimator/ransac_iterations", ransac_iterations_);

    
}

/*
Control block to perform surface estimaton when required by the configuration parameters. 
*/
void SurfaceEstimator::estimateSurface(const int idx){
    double ts; 
    if (!pose_graph_.timeQuery(idx, ts))
        return; // Not available!

    auto cloud_ptr = cloudQuery(ts - 0.5*window_size_, ts + 0.5*window_size_);
    if (cloud_ptr->size() < min_inlier_count_)
        return;

    downSample(cloud_ptr);
    if (cloud_ptr->size() < min_inlier_count_)
        return;

    if (!fitPlane(cloud_ptr))
        return;

    // Woho
    pose_graph_.surfaceCallback(idx, plane_coeffs_.cast<double>());
}




pcl::PointCloud<pcl::PointXYZ>::Ptr SurfaceEstimator::cloudQuery(const double t0, const double t1) const {
    // Find bounds in point buffer
    auto start = point_buffer_.lowerBound(t0);
    auto end = point_buffer_.lowerBound(t1);

    // Allocate a new PCL point cloud
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(start.distance_to(end));  // Efficient pre-allocation

    for (auto it = start; it != end; ++it) {
        cloud->emplace_back(it->x, it->y, it->z);
    }

    return cloud;
}

void SurfaceEstimator::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    if (!cloud || cloud->empty()) 
        return;

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.setInputCloud(cloud);

    auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel_grid.filter(*filtered_cloud);

    *cloud = std::move(*filtered_cloud); // Modify in place to avoid pointer swapping
}


bool SurfaceEstimator::fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return false;
    };

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_iterations_);
    seg.setDistanceThreshold(ransac_threshold_);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    if (inliers.indices.size() < min_inlier_count_) {
        return false; // Plane fitting failed
    }

    // Convert PCL coefficients to Eigen::Vector4f
    plane_coeffs_ = Eigen::Vector4f(
        coefficients.values[0],  // Normal X
        coefficients.values[1],  // Normal Y
        coefficients.values[2],  // Normal Z
        coefficients.values[3]   // Offset d
    );

    // Ensure a consistent normal direction
    if (plane_coeffs_[3] < 0) {
        plane_coeffs_ = -plane_coeffs_;
    }

    return true;
}