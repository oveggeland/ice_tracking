#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>
#include <open3d/t/geometry/PointCloud.h>


#include <opencv2/opencv.hpp>
#include <map>
#include <random>

// Defines an index in the raster
struct IdxType {
    int x;
    int y;

    // Allows emplacement
    IdxType(int x_, int y_) : x(x_), y(y_) {}
};

/*
This class implements a rasterized version of the pointclouds. This is useful for efficient radius and neighbor lookups.
*/
class CloudRaster{
public:
    // Construct from pointcloud
    CloudRaster(const open3d::t::geometry::PointCloud& pcd, float grid_size);

    // De-rasterize
    open3d::t::geometry::PointCloud toPointCloud() const;

    // Processing methods
    void cluster(size_t eps, size_t min_points);
    void smoothUniform(size_t window_size); // Uniform convolution of attributes
    void estimateDeformation(size_t window_size); // Estimate deformation (local elevation variance) 

    // Nice functionality
    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline int pointCount() const { return occupied_.size(); }

    inline bool hasIntensity() const { return intensity_.size() > 0; }
    inline bool hasDeformation() const { return deformation_.size() > 0; }

private:
    // Grid definition
    float x_min_, y_min_;          // Coordinates of corner of raster region
    int width_, height_;            // Number of cells on each axis
    float grid_size_;              // Metric distance between grid cells

    void defineGrid(const Eigen::Matrix2Xf& xy);

    // Data grid
    Eigen::MatrixXi idx_;
    Eigen::MatrixXi count_;             // Number of points, falling within a cell during initialization
    Eigen::MatrixXf elevation_;         // Mean elevation within a cell
    Eigen::MatrixXf intensity_;         // Mean intensity
    Eigen::MatrixXf deformation_;       // Estimated deformation

    // Keep track of occupied cells
    std::vector<IdxType> occupied_;

    // Helpers
    std::vector<int> findNeighbors(const IdxType& idx, const size_t eps);
    bool expandCluster(std::vector<int>& labels, const int idx, int cluster_id, size_t eps, size_t min_points);

    void smoothPoint(const IdxType& idx, const size_t window_size);
    void estimatePointDeformation(const IdxType& idx, const size_t window_size);
};