#include "RasterizedCluster.h"

void RasterizedCluster::expandCluster(const int idx){
    int& label = super_labels_[idx];
    if (label > 0)
        return; // Already been here
    label = cluster_id_;
    super_clusters_[cluster_id_].push_back(idx);
    
    // Find neighbors
    std::vector<int> neighbors = raster_.findNeighbors(idx, eps_);
    
    // Core or edge?
    if (neighbors.size() < min_points_)
        return; // I am edge, do not keep expanding
    
    for (const int& neighbor_idx: neighbors)
        expandCluster(neighbor_idx);
}


void RasterizedCluster::runClustering(){
    const size_t n_pixels = raster_.cellCount();
    super_labels_ = std::vector<int>(n_pixels, 0);

    for (int i = 0; i < n_pixels; ++i){
        if (super_labels_[i] != 0)
            continue; // Already labelled
        
        const std::vector<int> neighbors = raster_.findNeighbors(i, eps_);
        if (neighbors.size() < min_points_)
            super_labels_[i] = -1;
        else{
            super_clusters_[cluster_id_];
            expandCluster(i);
            cluster_id_ ++;
        }       
    }

    retrace();
}


void RasterizedCluster::retrace(){
    const std::vector<std::vector<int>>& trace = raster_.pointTrace();
    labels_.resize(raster_.pointCount(), -1); // Initialize labels to -1

    // For each cluster
    for (const auto& [id, super_cluster] : super_clusters_){
        // Insert a new vector in clusters_ with correct id
        clusters_[id];
        auto& retraced_cluster = clusters_[id];
        
        // Iterate over each cell (super pixel) in raster
        for (int super_pixel : super_cluster){
            // Get trace from superpixel to point cloud indices
            const auto& pixel_trace = trace[super_pixel];  
            
            // Retrace cluster indices
            retraced_cluster.insert(retraced_cluster.end(), pixel_trace.begin(), pixel_trace.end());

            // Set labels accordingly
            for (const int& idx: pixel_trace)
                labels_[idx] = id;
        }
    }
}


void RasterizedCluster::keepLargest(const int n_clusters) {
    if (n_clusters >= clusters_.size())
        return;

    // Sort by size (in raster scale)
    std::multimap<int, int> cluster_sizes;
    for (const auto& [cluster_id, cluster] : super_clusters_) {
        cluster_sizes.emplace(cluster.size(), cluster_id);
    }

    // Remove the smallest clusters
    for (const auto& [size, id] : cluster_sizes) {
        if (clusters_.size() <= n_clusters)
            break;
        removeCluster(id);
    }
}


void RasterizedCluster::removeCluster(const int cluster_id){
    ROS_INFO_STREAM("Removing cluster " << cluster_id);
    // First set label to -1
    for (const int& i: clusters_[cluster_id])
        labels_[i] = -1;
    for (const int& i: super_clusters_[cluster_id])
        super_labels_[i] = -1;
    
    // Erase from containers
    clusters_.erase(cluster_id);
    super_clusters_.erase(cluster_id);
}


std::vector<int> RasterizedCluster::getNoise(){
    std::vector<int> noise;
    noise.reserve(labels_.size());

    for (int i = 0; i < labels_.size(); ++i){
        if (labels_[i] == -1)
            noise.push_back(i);
    }
    return noise;
}


void RasterizedCluster::visualizeClusters(){
    // Create an image to visualize clusters
    cv::Mat cluster_image(raster_.height(), raster_.width(), CV_8UC3, cv::Scalar(0, 0, 0)); // Black background
    ROS_INFO_STREAM("Size of image: " << raster_.height() << ", " << raster_.width());

    // Generate random colors for each cluster
    std::map<int, cv::Vec3b> cluster_colors;
    std::mt19937 rng(123); // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist(0, 255);

    cluster_colors[-1] = cv::Vec3b(255, 255, 255);

    const auto& cells = raster_.cells();
    for (int i = 0; i < cells.size(); ++i) {
        const int& label = super_labels_[i];

        if (cluster_colors.find(label) == cluster_colors.end()) {
            cluster_colors[label] = cv::Vec3b(dist(rng), dist(rng), dist(rng));
        }

        const auto cell = cells[i];  // Get pixel position
        cluster_image.at<cv::Vec3b>(cell.y, cell.x) = cluster_colors[label];
    }    

    // Display the result
    double scale = 500.0 / std::max<int>(raster_.height(), raster_.width());
    cv::resize(cluster_image, cluster_image, cv::Size(), scale, scale);
    cv::imshow("Cluster Visualization", cluster_image);
    cv::waitKey(0);
}