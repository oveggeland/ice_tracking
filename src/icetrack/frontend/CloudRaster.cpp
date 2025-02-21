#include "CloudRaster.h"


std::tuple<double, double, double, double> getBounds(const Eigen::Matrix2Xd& xy) {
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < xy.cols(); ++i) {
        const double& x = xy(0, i);
        const double& y = xy(1, i);

        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }

    return {x_min, x_max, y_min, y_max};
}


void CloudRaster::setGridDefinition(const Eigen::Matrix3Xd& xyz){
    // Get bounds of dataset
    auto [x_min, x_max, y_min, y_max] = getBounds(xyz.topRows<2>());

    // Set minimum values
    x_min_ = x_min;
    y_min_ = y_min;

    // Set size of raster
    width_ = ceil((x_max-x_min) / grid_size_);
    height_ = ceil((y_max-y_min) / grid_size_);
}


CloudRaster::CloudRaster(const open3d::t::geometry::PointCloud& pcd, double grid_size) : grid_size_(grid_size){
    ROS_INFO_STREAM("Constructing raster from cloud");

    // Map pcd data to Eigen format
    int num_points = pcd.GetPointPositions().GetShape(0);
    double* data_ptr = pcd.GetPointPositions().Contiguous().GetDataPtr<double>();

    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> xyz(data_ptr, 3, num_points);
    Eigen::Matrix2Xd xy = xyz.topRows<2>();
    Eigen::VectorXd z = xyz.row(2);

    // Define grid layout (offset, size, etc.)
    setGridDefinition(xyz);

    // Initialize data matrices
    count_ = Eigen::MatrixXi::Constant(height_, width_, 0);
    elevation_ = Eigen::MatrixXd::Constant(height_, width_, 0); // No need for zero initialize?

    // Reserve space in index vector
    occupied_.reserve(num_points);


    ROS_INFO_STREAM("Iterate over all points to fill in matrices");
    for (int i = 0; i < num_points; ++i){
        // Get point coordinate in raster
        const int x = static_cast<int>((xy(0, i) - x_min_)/grid_size_);
        const int y = static_cast<int>((xy(1, i) - y_min_)/grid_size_);
        
        if (0 == count_(y, x)){ // Check for first allocation to a grid cell
            occupied_.emplace_back(x, y);
            elevation_(y, x) = z(i);
        }
        else{
            elevation_(y, x) += z(i);
        }

        ++count_(y, x);
    }


    // Calcualate mean values
    for (const auto& idx: occupied_){
        elevation_(idx.y, idx.x) /= count_(idx.y, idx.x);
    }
}


open3d::geometry::PointCloud CloudRaster::toPointCloud() const {
    open3d::geometry::PointCloud cloud;
    cloud.points_.reserve(occupied_.size());

    // Define center of origin cell
    double x0 = x_min_ + 0.5*grid_size_;
    double y0 = y_min_ + 0.5*grid_size_;

    // Iterate through all occupied cells and add coordinates to cloud
    for (const auto& idx: occupied_){
        const double x = x0 + idx.x*grid_size_;
        const double y = y0 + idx.y*grid_size_;
        const double z = elevation_(idx.y, idx.x);
        cloud.points_.emplace_back(x, y, z);
    }

    return cloud;
}