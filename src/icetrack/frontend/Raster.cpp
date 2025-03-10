#include "Raster.h"

std::tuple<double, double, double, double> getBounds(const std::vector<Eigen::Vector3d>& points) {
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (const Eigen::Vector3d& p: points) {
        const double& x = p.x();
        const double& y = p.y();

        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }

    return {x_min, x_max, y_min, y_max};
}


void Raster::defineGrid(const std::vector<Eigen::Vector3d>& points){
    // Get bounds of dataset
    const auto [x_min, x_max, y_min, y_max] = getBounds(points);

    // Set x-axis params
    x0_ = x_min - 0.5 * grid_size_;
    width_ = 1 + (x_max - x0_) / grid_size_;

    // Y-axis params
    y0_ = y_min - 0.5 * grid_size_;
    height_ = 1 + (y_max - y0_) / grid_size_;
}


RasterCell Raster::getCell(const Eigen::Vector3d& p) const{
    return getCell(Eigen::Vector2d(p.x(), p.y()));
}

RasterCell Raster::getCell(const Eigen::Vector2d& p) const{
    return RasterCell{
        static_cast<int>((p.x() - x0_) / grid_size_),
        static_cast<int>((p.y() - y0_) / grid_size_)
    };
}


Raster::Raster(const std::vector<Eigen::Vector3d>& points, const double grid_size) : points_(points), grid_size_(grid_size){
    if (points.empty())
        return;

    // Define grid layout (offset, size, etc.)
    defineGrid(points);
    raster_ = Eigen::MatrixXi::Constant(height_, width_, -1);

    // Reserve space in index vector
    cells_.reserve(height_*width_);
    point_trace_.reserve(height_*width_);

    // Sum up cell attributes for each point in cloud
    const int num_points = points.size();
    for (int i = 0; i < num_points; ++i){
        // Get point coordinate in raster
        const RasterCell cell = getCell(points[i]);
        
        // Check if cell is previously assigned
        int& cell_idx = raster_(cell.v, cell.u);
        if (cell_idx == -1){ 
            // First time this cell is assigned
            cell_idx = cells_.size();
            cells_.push_back(cell);

            // Initialize new point trace vector for the cell
            point_trace_.emplace_back(1, i);
        }
        else{
            // Push point trace
            point_trace_[cell_idx].push_back(i);
        }
    }
}


Eigen::Vector2d Raster::getVector(const RasterCell& cell) const{
    return Eigen::Vector2d(
        x0_ + cell.u*grid_size_,
        y0_ + cell.v*grid_size_
    );
}

bool Raster::inBounds(const RasterCell& cell) const {
    return (cell.u > 0 && cell.u < width_ && cell.v > 0 && cell.v < height_);
}

bool Raster::isOccupied(const RasterCell& cell) const {
    if (!inBounds(cell))
        return false;
    return (raster_(cell.v, cell.u) != -1);
}

double Raster::intersection(const Raster& other) const{
    double area = 0;
    for (const auto& cell: cells_){
        const Eigen::Vector2d pos = getVector(cell);
        
        auto other_cell = other.getCell(getVector(cell));
        if (other.isOccupied(other_cell))
            area += grid_size_*grid_size_;
    }

    return area;
}

// Create a new raster by expanding with new points
void Raster::expand(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty())
        return;

    // Temp save some current config
    const double x0_old = x0_;
    const double y0_old = y0_;
    const int idx0 = points_.size(); // Start iterating from here later

    // Insert new points
    points_.insert(points_.end(), points.begin(), points.end());
    
    // Redefine grid
    defineGrid(points_);

    // Find shift between new and old raster
    int x_shift = (x0_old - x0_) / grid_size_;
    int y_shift = (y0_old - y0_) / grid_size_;
    
    // Copy old raster
    Eigen::MatrixXi new_raster = Eigen::MatrixXi::Constant(height_, width_, -1);
    new_raster.block(y_shift, x_shift, raster_.rows(), raster_.cols()) = raster_;
    raster_ = new_raster; 

    // Shift cell list
    for (auto& cell: cells_){
        cell.u += x_shift;
        cell.v += y_shift;
    }

    // Reserve more space
    cells_.reserve(width_*height_);
    point_trace_.reserve(width_*height_);

    for (int i = idx0; i < points_.size()-1; ++i){
        // Get point coordinate in raster
        const RasterCell cell = getCell(points_[i]);

        // Fill in cell
        int& cell_idx = raster_(cell.v, cell.u);
        if (cell_idx == -1){ 
            cell_idx = cells_.size();
            cells_.push_back(cell);

            point_trace_.push_back(std::vector<int>(1, i));
        }
        else{
            // Push point trace
            point_trace_[cell_idx].push_back(i);
        }
    };
};


std::vector<int> Raster::findNeighbors(const RasterCell& cell, const int eps) const{
    std::vector<int> neighbors;
    neighbors.reserve((2*eps + 1)*(2*eps + 1));

    // Define neighborhood for searching
    const int u0 = std::max<int>(0, cell.u - eps);
    const int u1 = std::min<int>(width_, cell.u + eps + 1);

    const int v0 = std::max<int>(0, cell.v - eps);
    const int v1 = std::min<int>(height_, cell.v + eps + 1);

    // Iterate in neighborhood and push back neighbors
    for (int u = u0; u < u1; ++u){
        for (int v = v0; v < v1; ++v){
            const int cell_idx = raster_(v, u);
            if (cell_idx != -1)
                neighbors.push_back(cell_idx);
        }
    }
    return neighbors;
}