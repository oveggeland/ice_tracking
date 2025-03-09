#pragma once

#include <ros/ros.h>

#include <vector>
#include <Eigen/Dense>

struct RasterCell {
    int x;
    int y;

    // Allows emplacement
    RasterCell(int x_, int y_) : x(x_), y(y_) {}
};

class Raster{
public:
    // Construct from 3D points
    Raster(const std::vector<Eigen::Vector3d>& points, const double grid_size=0.2);

    // Expanding with new points
    Raster expand(const std::vector<Eigen::Vector3d>& points) const;

    // Neighbor search
    std::vector<int> findNeighbors(const RasterCell& cell, const int eps);
    std::vector<int> findNeighbors(const int cell_idx, const int eps);

    // Accessors
    int cellCount() const { return cells_.size(); }
    int pointCount() const { return point_count_; }
    int width() const { return width_; }
    int height() const { return height_; }

    const Eigen::MatrixXi& raster() const { return raster_; }
    const std::vector<RasterCell>& cells() const { return cells_; }
    const std::vector<std::vector<int>>& pointTrace() const { return point_trace_; }

private:
    std::vector<Eigen::Vector3d> points_;

    // Grid definition
    float x_min_, y_min_;          // Coordinates of corner of raster region
    int width_, height_;            // Number of cells on each axis
    float grid_size_;              // Metric distance between grid cells

    int point_count_;

    void defineGrid(const std::vector<Eigen::Vector3d>& points);

    // Data structures
    Eigen::MatrixXi raster_;                        // Raster grid
    std::vector<RasterCell> cells_;                 // Coordinates of occupied cells
    std::vector<std::vector<int>> point_trace_;     // Trace back from cell to original points

    // Helpers
    RasterCell getCell(const Eigen::Vector3d& p);
};