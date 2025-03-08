#include "ClusterRaster.h"


std::tuple<float, float, float, float> getBounds(const std::vector<Eigen::Vector3d>& points) {
    float x_min = std::numeric_limits<float>::infinity();
    float x_max = -std::numeric_limits<float>::infinity();
    float y_min = std::numeric_limits<float>::infinity();
    float y_max = -std::numeric_limits<float>::infinity();

    for (const Eigen::Vector3d& p: points) {
        const float& x = p.x();
        const float& y = p.y();

        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }

    return {x_min, x_max, y_min, y_max};
}


void ClusterRaster::defineGrid(const std::vector<Eigen::Vector3d>& points){
    // Get bounds of dataset
    auto [x_min, x_max, y_min, y_max] = getBounds(points);

    // Set x-axis params
    if (x_min == x_max){
        x_min_ = x_min - 0.5*grid_size_; 
        width_ = 1;
    }
    else {
        x_min_ = x_min;
        width_ = ceil((x_max-x_min) / grid_size_);
    }

    if (y_min == y_max){
        y_min_ = y_min - 0.5*grid_size_;
        height_ = 1;
    }
    else {
        y_min_ = y_min;
        height_ = ceil((y_max-y_min) / grid_size_);
    }
}


ClusterRaster::ClusterRaster(const std::vector<Eigen::Vector3d>& points, float grid_size) : grid_size_(grid_size){
    const int num_points = points.size();

    // Define grid layout (offset, size, etc.)
    defineGrid(points);
    idx_ = Eigen::MatrixXi::Constant(height_, width_, -1);

    // Reserve space in index vector
    cells_.reserve(num_points);
    cell_trace_.reserve(num_points);

    // Sum up cell attributes for each point in cloud
    for (int i = 0; i < num_points; ++i){
        const Eigen::Vector3d& p = points[i];

        // Get point coordinate in raster
        const int x = static_cast<int>((p.x() - x_min_)/grid_size_);
        const int y = static_cast<int>((p.y() - y_min_)/grid_size_);
        

        int& idx = idx_(y, x);
        if (idx == -1){ 
            // First time this cell is assigned
            idx = cells_.size();

            // Track occupied cells in vector
            cells_.emplace_back(x, y);
            cell_trace_.emplace_back(1, i);
        }
        else{
            // Add to cell_trace
            cell_trace_[idx].push_back(i);
        }
    }
    cluster();
    retrace();
}

// NB, this is also returning "self" as neighbor. 
std::vector<int> ClusterRaster::findNeighbors(const IdxType& idx){
    std::vector<int> neighbors;
    neighbors.reserve(pow((2*eps_+1), 2));

    for (int x = std::max<int>(0, idx.x - eps_); x < std::min<int>(width_, idx.x + eps_ + 1); ++x){
        for (int y = std::max<int>(0, idx.y - eps_); y < std::min<int>(height_, idx.y + eps_ + 1); ++y){
            const int& idx = idx_(y, x);
            if (idx != -1)
                neighbors.push_back(idx);
        }
    }
    return neighbors;
}

// Idx is not noise
void ClusterRaster::expandCluster(const int idx){
    if (labels_[idx] > 0)
        return; // Already been here
    labels_[idx] = cluster_id_;
    clusters_.back().push_back(idx);
    
    // Find neighbors
    std::vector<int> neighbors = findNeighbors(cells_[idx]);
    
    // Core or edge?
    if (neighbors.size() < min_points_)
        return; // I am edge, do not keep expanding
    
    for (const int& neighbor_idx: neighbors)
        expandCluster(neighbor_idx);
}

void ClusterRaster::cluster(){
    const size_t n_pixels = cells_.size();
    labels_ = std::vector<int>(n_pixels, 0);

    for (int i = 0; i < n_pixels; ++i){
        if (labels_[i] != 0)
            continue; // Already labelled
        
        const std::vector<int> neighbors = findNeighbors(cells_[i]);
        if (neighbors.size() < min_points_)
            labels_[i] = -1;
        else{
            clusters_.emplace_back();
            expandCluster(i);
            cluster_id_ ++;
        }       
    }
}
    
void ClusterRaster::visualizeClusters(){
    // Create an image to visualize clusters
    cv::Mat cluster_image(height_, width_, CV_8UC3, cv::Scalar(0, 0, 0)); // Black background

    // Generate random colors for each cluster
    std::map<int, cv::Vec3b> cluster_colors;
    std::mt19937 rng(1234); // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 255);

    for (int i = 0; i < cells_.size(); ++i) {
        int label = labels_[i];
        if (label > 0) {  // Ignore noise (-1) and unvisited points (0)
            if (cluster_colors.find(label) == cluster_colors.end()) {
                cluster_colors[label] = cv::Vec3b(dist(rng), dist(rng), dist(rng));    visualizeClusters();
            }

            IdxType idx = cells_[i];  // Get pixel position
            cluster_image.at<cv::Vec3b>(idx.y, idx.x) = cluster_colors[label];
        }
    }

    // Display the result
    cv::imshow("Cluster Visualization", cluster_image);
    cv::waitKey(0);
}


void ClusterRaster::retrace(){
    std::vector<std::vector<int>> retraced_clusters;
    
    // For each cluster
    for (const auto& cluster : clusters_){
        retraced_clusters.emplace_back();
        auto& retraced_cluster = retraced_clusters.back();
        
        for (int superpixel : cluster){
            const auto& pixel_trace = cell_trace_[superpixel];  
            retraced_cluster.insert(retraced_cluster.end(), pixel_trace.begin(), pixel_trace.end());
        }
    }
    
    clusters_ = std::move(retraced_clusters); // Replace old clusters
}