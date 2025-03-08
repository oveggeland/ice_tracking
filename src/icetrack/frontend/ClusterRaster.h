#pragma once

#include <ros/ros.h>

#include <Eigen/Dense>

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
class ClusterRaster{
public:
    // Construct from 3D points
    ClusterRaster(const std::vector<Eigen::Vector3d>& points, float grid_size);

    // Nice functionality
    inline int width() const { return width_; }
    inline int height() const { return height_; }
    inline int pointCount() const { return cells_.size(); }

    // Get clusters
    std::vector<std::vector<int>> getClusters() { return clusters_; }

private:
    // Grid definition
    float x_min_, y_min_;          // Coordinates of corner of raster region
    int width_, height_;            // Number of cells on each axis
    float grid_size_;              // Metric distance between grid cells

    void defineGrid(const std::vector<Eigen::Vector3d>& points);

    // Raster of indices (traces to cells_)
    Eigen::MatrixXi idx_;

    std::vector<IdxType> cells_; // Keep track of occupied cells
    std::vector<std::vector<int>> cell_trace_; // Trace back from cell to original points

    int cluster_id_ = 1;
    size_t eps_ = 3;
    size_t min_points_ = 25;

    std::vector<int> labels_;
    std::vector<std::vector<int>> clusters_;

    // Helpers
    std::vector<int> findNeighbors(const IdxType& idx);
    void expandCluster(const int idx);
    void cluster();
    void retrace();

    void visualizeClusters();
};