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


void CloudRaster::defineGrid(const Eigen::Matrix2Xd& xy){
    // Get bounds of dataset
    auto [x_min, x_max, y_min, y_max] = getBounds(xy);

    // Set minimum values
    x_min_ = x_min;
    y_min_ = y_min;

    // Set size of raster
    width_ = ceil((x_max-x_min) / grid_size_);
    height_ = ceil((y_max-y_min) / grid_size_);
}


CloudRaster::CloudRaster(const open3d::t::geometry::PointCloud& pcd, double grid_size) : grid_size_(grid_size){
    int num_points = pcd.GetPointPositions().GetShape(0);

    // Map positions
    double* position_ptr = pcd.GetPointPositions().Contiguous().GetDataPtr<double>();

    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>> xyz(position_ptr, 3, num_points);
    Eigen::Matrix2Xd xy = xyz.topRows<2>();
    Eigen::VectorXd z = xyz.row(2);

    // Map intensities
    float* intensity_ptr = pcd.GetPointAttr("intensities").Contiguous().GetDataPtr<float>();

    Eigen::Map<Eigen::VectorXf> intensities(intensity_ptr, 1, num_points);

    // Define grid layout (offset, size, etc.)
    defineGrid(xy);

    // Initialize data matrices
    count_ = Eigen::MatrixXi::Constant(height_, width_, 0);
    elevation_ = Eigen::MatrixXd(height_, width_);
    intensity_ = Eigen::MatrixXf(height_, width_);

    // Reserve space in index vector
    occupied_.reserve(num_points);

    // Sum up cell attributes for each point in cloud
    for (int i = 0; i < num_points; ++i){
        // Get point coordinate in raster
        const int x = static_cast<int>((xy(0, i) - x_min_)/grid_size_);
        const int y = static_cast<int>((xy(1, i) - y_min_)/grid_size_);
        
        if (0 == count_(y, x)++){ // Check for first allocation to a grid cell
            occupied_.emplace_back(x, y);

            elevation_(y, x) = z(i);
            intensity_(y, x) = intensities(i);
        }
        else{
            elevation_(y, x) += z(i);
            intensity_(y, x) += intensities(i);
        }
    }

    // Average attributes
    for (const auto& idx: occupied_){
        const int& cnt = count_(idx.y, idx.x);

        elevation_(idx.y, idx.x) /= cnt;
        intensity_(idx.y, idx.x) /= cnt;
    }
}

void CloudRaster::smoothPoint(const IdxType& idx, const size_t window_size){
    const size_t offset = window_size / 2;

    // Clamp indices to stay within bounds
    const int row0 = std::clamp(idx.y - static_cast<int>(offset), 0, height_ - 1);
    const int row1 = std::clamp(idx.y + static_cast<int>(offset), 0, height_ - 1);
    const int col0 = std::clamp(idx.x - static_cast<int>(offset), 0, width_ - 1);
    const int col1 = std::clamp(idx.x + static_cast<int>(offset), 0, width_ - 1);

    // Compute block sizes correctly (inclusive range)
    const int block_rows = row1 - row0 + 1;
    const int block_cols = col1 - col0 + 1;

    // Extract submatrix safely
    const auto& count_block = count_.block(row0, col0, block_rows, block_cols);
    const auto& z_block = elevation_.block(row0, col0, block_rows, block_cols);
    const auto& i_block = intensity_.block(row0, col0, block_rows, block_cols);

    double z_sum = 0;
    double i_sum = 0;
    int cnt_sum = 0;
    for (int r = 0; r < block_rows; ++r){
        for (int c = 0; c < block_cols; ++c){
            const int cnt = count_block(r, c);
            if (count_block(r, c) > 0){
                z_sum += cnt*z_block(r, c);
                i_sum += cnt*i_block(r, c);
                cnt_sum += cnt;
            }
        }
    }
    
    elevation_(idx.y, idx.x) = z_sum / cnt_sum;
    intensity_(idx.y, idx.x) = i_sum / cnt_sum;
}


void CloudRaster::smoothUniform(size_t window_size){
    for (const auto& idx: occupied_){
        smoothPoint(idx, window_size);
    }
}

void CloudRaster::estimatePointDeformation(const IdxType& idx, const size_t window_size){
    const size_t offset = window_size / 2;

    // Clamp indices to stay within bounds
    const int row0 = std::clamp(idx.y - static_cast<int>(offset), 0, height_ - 1);
    const int row1 = std::clamp(idx.y + static_cast<int>(offset), 0, height_ - 1);
    const int col0 = std::clamp(idx.x - static_cast<int>(offset), 0, width_ - 1);
    const int col1 = std::clamp(idx.x + static_cast<int>(offset), 0, width_ - 1);

    // Compute block sizes correctly (inclusive range)
    const int block_rows = row1 - row0 + 1;
    const int block_cols = col1 - col0 + 1;

    // Extract submatrix safely
    const auto& count_block = count_.block(row0, col0, block_rows, block_cols);
    const auto& z_block = elevation_.block(row0, col0, block_rows, block_cols);

    double z_sum = 0;
    double z2_sum = 0;
    int cnt = 0;
    for (int r = 0; r < block_rows; ++r){
        for (int c = 0; c < block_cols; ++c){
            if (count_block(r, c) > 0){
                const double& z = z_block(r, c);
                z_sum += z;
                z2_sum += z*z;
                ++cnt;
            }
        }
    }
    
    const double mean = z_sum / cnt;
    deformation_(idx.y, idx.x) = z2_sum / cnt  - mean*mean;
}


void CloudRaster::estimateDeformation(size_t window_size){
    // Initialize matrix
    deformation_ = Eigen::MatrixXf(height_, width_);

    // Iterate over points and do individual estimation of deformation
    for (const auto& idx: occupied_){
        estimatePointDeformation(idx, window_size);
    }
}


open3d::t::geometry::PointCloud CloudRaster::toPointCloud() const {
    int num_points = pointCount();

    // Allocate memory for positions and attributes
    std::vector<float> positions;
    std::vector<float> intensities;
    std::vector<float> deformation;
    positions.reserve(3*num_points);
    intensities.reserve(num_points);
    deformation.reserve(num_points);

    // Define center of origin cell
    double x0 = x_min_ + 0.5*grid_size_;
    double y0 = y_min_ + 0.5*grid_size_;

    // Iterate through all occupied cells and add coordinates to cloud
    for (const auto& idx: occupied_){
        // Add points
        positions.push_back(x0 + idx.x*grid_size_);
        positions.push_back(y0 + idx.y*grid_size_);
        positions.push_back(elevation_(idx.y, idx.x));

        // Add attributes
        intensities.push_back(intensity_(idx.y, idx.x));
        deformation.push_back(deformation_(idx.y, idx.x));
    }

    // Move ownership to tensor cloud
    open3d::t::geometry::PointCloud cloud;
    cloud.SetPointPositions(open3d::core::Tensor(std::move(positions), {num_points, 3}, open3d::core::Dtype::Float32));
    cloud.SetPointAttr("intensities", open3d::core::Tensor(std::move(intensities), {num_points, 1}, open3d::core::Dtype::Float32));
    cloud.SetPointAttr("deformation", open3d::core::Tensor(std::move(deformation), {num_points, 1}, open3d::core::Dtype::Float32));

    return cloud;
}