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
    x_min_ = std::floor(x_min / grid_size_) * grid_size_;
    width_ = 1 + (x_max - x_min_) / grid_size_;

    // Y-axis params
    y_min_ = std::floor(y_min / grid_size_) * grid_size_;
    height_ = 1 + (y_max - y_min_) / grid_size_;
}



RasterCell Raster::getCell(const Eigen::Vector3d& p){
    return RasterCell{
        static_cast<int>((p.x() - x_min_) / grid_size_),
        static_cast<int>((p.y() - y_min_) / grid_size_)
    };
}


Raster::Raster(const std::vector<Eigen::Vector3d>& points, const double grid_size) : grid_size_(grid_size){
    point_count_ = points.size();
    if (points.empty())
        return;

    // Define grid layout (offset, size, etc.)
    defineGrid(points);
    raster_ = Eigen::MatrixXi::Constant(height_, width_, -1);

    // Reserve space in index vector
    const int num_points = points.size();
    cells_.reserve(num_points);
    point_trace_.reserve(num_points);

    // Sum up cell attributes for each point in cloud
    for (int i = 0; i < num_points; ++i){
        // Get point coordinate in raster
        const RasterCell cell = getCell(points[i]);
        
        // Check if cell is previously assigned
        int& cell_idx = raster_(cell.y, cell.x);
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

// Create a new raster by expanding with new points
Raster Raster::expand(const std::vector<Eigen::Vector3d>& points) const {
    if (points.empty())
        return Raster(points_);
        
    // Create a new vector containing both existing and new points
    std::vector<Eigen::Vector3d> expanded_points = points_; // Assuming `points_` is a member
    expanded_points.insert(expanded_points.end(), points.begin(), points.end());

    // Return a new Raster object with the expanded point set
    return Raster(expanded_points);
};


// NB, this is also returning "self" as neighbor. 
std::vector<int> Raster::findNeighbors(const RasterCell& cell, const int eps){
    std::vector<int> neighbors;
    neighbors.reserve(pow((2*eps+1), 2));

    for (int x = std::max<int>(0, cell.x - eps); x < std::min<int>(width_, cell.x + eps + 1); ++x){
        for (int y = std::max<int>(0, cell.y - eps); y < std::min<int>(height_, cell.y + eps + 1); ++y){
            const int& cell_idx = raster_(y, x);
            if (cell_idx != -1)
                neighbors.push_back(cell_idx);
        }
    }
    return neighbors;
}


// NB, this is also returning "self" as neighbor. 
std::vector<int> Raster::findNeighbors(const int cell_idx, const int eps){
    return findNeighbors(cells_[cell_idx], eps);
}