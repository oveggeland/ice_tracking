#pragma once

#include <ros/ros.h>

#include <vector>
#include <Eigen/Dense>

// Struct for raster position
struct RasterCell {
    int u;
    int v;

    // Allows emplacement
    RasterCell(int u_, int v_) : u(u_), v(v_) {}
};

// Main class for rasters
class Raster{
public:
    // Construct from 3D points
    Raster(const std::vector<Eigen::Vector3d>& points, const double grid_size=0.5);

    // Expand raster with new points
    void expand(const std::vector<Eigen::Vector3d>& points);

    // Estimate area of intersection between this and other
    double intersection(const Raster& other) const;

    // Neighbor search
    std::vector<int> findNeighbors(const RasterCell& cell, const int eps) const;
    std::vector<int> findNeighbors(const int cell_idx, const int eps) const{
        return findNeighbors(cells_[cell_idx], eps);
    };

    // Accessors
    int width() const { return width_; }
    int height() const { return height_; }
    int numPoints() const { return points_.size(); }

    int gridCellCount() const { return width() * height(); }
    int numOccupiedCells() const { return cells_.size(); }

    double cellArea() const { return std::pow(grid_size_, 2); }
    double gridArea() const { return gridCellCount() * cellArea(); }
    double occupiedArea() const { return numOccupiedCells() * cellArea(); }

    const Eigen::MatrixXi& raster() const { return raster_; }
    const std::vector<RasterCell>& cells() const { return cells_; }
    const std::vector<std::vector<int>>& pointTrace() const { return point_trace_; }

private:
    // Source points
    std::vector<Eigen::Vector3d> points_;

    // Grid definition
    double x0_, y0_;                // Position of corner edge
    int width_, height_;            // Number of cells on each axis
    double grid_size_;              // Metric distance between grid cells

    void defineGrid(const std::vector<Eigen::Vector3d>& points);

    // Data structures
    Eigen::MatrixXi raster_;                        // Raster grid
    std::vector<RasterCell> cells_;                 // Coordinates of occupied cells
    std::vector<std::vector<int>> point_trace_;     // Trace from cell to original point indices

    // Helpers
    bool inBounds(const RasterCell& cell) const;
    bool isOccupied(const RasterCell& cell) const;
    Eigen::Vector2d getVector(const RasterCell& cell) const;
    RasterCell getCell(const Eigen::Vector2d& p) const;
    RasterCell getCell(const Eigen::Vector3d& p) const;
};