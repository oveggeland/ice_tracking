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
    x_min_ = x_min - 0.5 * grid_size_;
    width_ = 1 + (x_max - x_min_) / grid_size_;

    // Y-axis params
    y_min_ = y_min - 0.5 * grid_size_;
    height_ = 1 + (y_max - y_min_) / grid_size_;
}


RasterCell Raster::getCell(const Eigen::Vector3d& p){
    return RasterCell{
        static_cast<int>((p.x() - x_min_) / grid_size_),
        static_cast<int>((p.y() - y_min_) / grid_size_)
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
void Raster::expand(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty())
        return;

    // Temp save some current config
    const int x_min_old = x_min_;
    const int y_min_old = y_min_;
    const int idx0 = points_.size(); // Start iterating from here later

    // Insert new points
    points_.insert(points_.end(), points.begin(), points.end());
    
    // Redefine grid
    defineGrid(points_);

    // Find shift between new and old raster
    int x_shift = (x_min_old - x_min_) / grid_size_;
    int y_shift = (y_min_old - y_min_) / grid_size_;
    
    // Copy old raster
    if (y_shift + raster_.rows() > height_ || x_shift + raster_.cols() > width_) {
        ROS_INFO_STREAM(x_min_old << ", " << y_min_old);
        ROS_INFO_STREAM(x_min_ << ", " << y_min_);
        ROS_INFO_STREAM(y_shift << ", " << x_shift);
        ROS_INFO_STREAM(raster_.rows() << ", " << raster_.cols());
        ROS_INFO_STREAM(height_ << ", " << width_);
        throw std::runtime_error("Shifting raster out of bounds.");
    }
    Eigen::MatrixXi new_raster = Eigen::MatrixXi::Constant(height_, width_, -1);
    new_raster.block(y_shift, x_shift, raster_.rows(), raster_.cols()) = raster_;
    raster_ = new_raster; 

    // Shift cell list
    for (auto& cell: cells_){
        cell.x += x_shift;
        cell.y += y_shift;
    }

    // Reserve more space
    cells_.reserve(width_*height_);
    point_trace_.reserve(width_*height_);

    for (int i = idx0; i < points_.size()-1; ++i){
        // Get point coordinate in raster
        const RasterCell cell = getCell(points_[i]);

        // Fill in cell
        int& cell_idx = raster_(cell.y, cell.x);
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