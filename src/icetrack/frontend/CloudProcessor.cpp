#include "frontend/CloudProcessor.h"

CloudProcessor::CloudProcessor(const ros::NodeHandle& nh){
    // Config
    getParamOrThrow(nh, "/mapping/grid_size", grid_size_);
    getParamOrThrow(nh, "/mapping/smoothing_radius", smoothing_radius_);
    getParamOrThrow(nh, "/mapping/deformation_radius", deformation_radius_);

    // Save clouds
    getParamOrThrow(nh, "/mapping/save_clouds", save_clouds_);
    if (save_clouds_){
        std::string ws = getParamOrThrow<std::string>(nh, "/workspace");
        std::string exp = getParamOrThrow<std::string>(nh, "/exp");
        cloud_path_ = joinPaths({ws, exp, "clouds/"});
        makePath(cloud_path_, true);
    }
}

open3d::t::geometry::PointCloud CloudProcessor::processCloud(open3d::t::geometry::PointCloud& pcd) const{
    // Clone the input point cloud
    open3d::t::geometry::PointCloud pcd_processed = pcd.Clone();

    // Downsample
    gridDownSample(pcd_processed);

    // Smooth elevation and intensity
    smoothCloud(pcd_processed);

    // Estimate deformation
    estimateLocalDeformation(pcd_processed);
    
    return pcd_processed;
}

// Pipeline functionality
void CloudProcessor::gridDownSample(open3d::t::geometry::PointCloud& pcd) const{
    // Copy z-value to scalar attribute and set to 0 in position tensor
    auto z_tensor = pcd.GetPointPositions().Slice(1, 2, 3);
    pcd.SetPointAttr("z_value", z_tensor.Clone());
    z_tensor.SetItem(open3d::core::Tensor::Zeros({1, 1}, open3d::core::Float32));

    // Downsample (now with z=0)
    pcd = pcd.VoxelDownSample(grid_size_);

    // Move z-value back to positions and remove attribute copy
    pcd.GetPointPositions().Slice(1, 2, 3) = pcd.GetPointAttr("z_value");
    pcd.RemovePointAttr("z_value");
};

void CloudProcessor::smoothCloud(open3d::t::geometry::PointCloud& pcd) const{
    // Convert to legacy 
    auto pcd_legacy = pcd.ToLegacy();
    int num_points = pcd_legacy.points_.size();

    // Build KDTree
    open3d::geometry::KDTreeFlann kdtree(pcd_legacy);

    // Allocate vector for smooth coordinates
    std::vector<float> z_smooth;
    z_smooth.reserve(num_points);

    // Loop over each point to compute the mean Z-coordinate
    for (size_t i = 0; i < num_points; ++i) {
        // Query point
        const Eigen::Vector3d& query_point = pcd_legacy.points_[i];

        // Perform radius search
        std::vector<int> indices;
        std::vector<double> distances;
        int num_neighbors = kdtree.SearchRadius(query_point, smoothing_radius_, indices, distances);

        double z_sum = 0;
        for (int idx : indices) {
            z_sum += pcd_legacy.points_[idx].z();
        }

        z_smooth.push_back(z_sum / num_neighbors);
    }

    pcd.GetPointPositions().Slice(1, 2, 3) = open3d::core::Tensor(std::move(z_smooth), {num_points, 1});
};

void CloudProcessor::estimateLocalDeformation(open3d::t::geometry::PointCloud& pcd) const{
    // Convert to legacy PointCloud
    open3d::geometry::PointCloud pcd_legacy = pcd.ToLegacy();
    int num_points = pcd_legacy.points_.size();

    // Build KDTree
    open3d::geometry::KDTreeFlann kdtree(pcd_legacy);

    // Allocate memory to save deformation values
    std::vector<float> deformation;
    deformation.reserve(num_points);

    // Loop over each point to calculate deformation
    for (size_t i = 0; i < pcd_legacy.points_.size(); ++i) {
        // Query point
        const Eigen::Vector3d& query_point = pcd_legacy.points_[i];

        // Perform radius search
        std::vector<int> indices;
        std::vector<double> distances;
        int num_neighbors = kdtree.SearchRadius(query_point, deformation_radius_, indices, distances);

        // Extract z-coordinates of neighbors
        double sum = 0;
        double sum2 = 0;

        for (int idx : indices) {
            double z = pcd_legacy.points_[idx].z();
            sum += z;
            sum2 += z*z;
        }

        double mean = sum / num_neighbors;
        double variance = sum2 / num_neighbors - mean*mean;

        // Store variance as deformation value
        deformation.push_back(variance);
    }

    pcd.SetPointAttr("deformation", open3d::core::Tensor(std::move(deformation), {num_points, 1}));
};



void CloudProcessor::estimatePlaneDeviations(open3d::t::geometry::PointCloud& pcd) const {
    // Perform RANSAC-based plane segmentation
    auto [plane_model, inliers] = pcd.SegmentPlane(0.2, 10, 100);

    // Extract plane coefficients (a, b, c, d)
    double a = plane_model[0].Item<double>();
    double b = plane_model[1].Item<double>();
    double c = plane_model[2].Item<double>();
    double d = plane_model[3].Item<double>();

    // Retrieve point cloud coordinates
    open3d::core::Tensor points = pcd.GetPointPositions(); // (N, 3) tensor

    // Compute signed distances from each point to the plane
    open3d::core::Tensor distances = 
        (points.Slice(1, 0, 1) * a + 
         points.Slice(1, 1, 2) * b + 
         points.Slice(1, 2, 3) * c + d) / 
        std::sqrt(a*a + b*b + c*c);

    // Store distances as a new attribute in the point cloud
    pcd.SetPointAttr("plane_dev", distances);

    std::cout << "Plane deviations added to point cloud." << std::endl;
}



void CloudProcessor::saveCloud(const open3d::t::geometry::PointCloud& pcd) const{
    std::stringstream fname;
    fname << std::fixed << static_cast<int>(ros::Time::now().toSec()) << ".ply";
    std::string fpath = joinPaths({cloud_path_, fname.str()});

    // Save the point cloud as a .ply file
    if (open3d::t::io::WritePointCloud(fpath, pcd))
        ROS_INFO_STREAM("Saved cloud at " << fpath);
    else
        ROS_WARN_STREAM("Failed to save cloud at " << fpath);
};
