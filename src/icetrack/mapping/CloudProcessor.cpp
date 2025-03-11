#include "frontend/CloudProcessor.h"

CloudProcessor::CloudProcessor(const ros::NodeHandle& nh){
    // Config
    getParamOrThrow(nh, "/cloud_processor/grid_size", grid_size_);
    getParamOrThrow(nh, "/cloud_processor/smoothing_window_size", smoothing_window_size_);
    getParamOrThrow(nh, "/cloud_processor/deformation_window_size", deformation_window_size_);

    // Save clouds
    getParamOrThrow(nh, "/cloud_processor/save_clouds", save_clouds_);
    if (save_clouds_){
        std::string ws = getParamOrThrow<std::string>(nh, "/workspace");
        std::string exp = getParamOrThrow<std::string>(nh, "/exp");
        cloud_path_ = joinPaths({ws, exp, "clouds/"});
        makePath(cloud_path_, true);
    }
}


open3d::t::geometry::PointCloud CloudProcessor::processCloud(const open3d::t::geometry::PointCloud& pcd) const{
    if (pcd.IsEmpty() || pcd.GetPointPositions().GetShape(0) == 0)
        return pcd; 

    CloudRaster raster(pcd, grid_size_);
    raster.smoothUniform(smoothing_window_size_);
    raster.estimateDeformation(deformation_window_size_);
    
    auto cloud = raster.toPointCloud();

    if (save_clouds_){
        saveCloud(cloud);
    }

    return cloud;
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
