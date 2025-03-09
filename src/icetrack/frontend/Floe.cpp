#include "frontend/Floe.h"

std::vector<int> Floe::findOutliers(){
    // Run clustering
    RasterizedCluster cluster(cloud_->points_);
    cluster.runClustering();

    // Keep only largest cluster
    cluster.keepLargest(1);
    return cluster.getNoise();
}


// Find points that can be associated to the Floe
std::vector<int> Floe::associatePoints(const std::vector<Eigen::Vector3d>& points){
    // Run cluster 
    RasterizedCluster cluster(cloud_->points_);
    cluster.runClustering();
    cluster.keepLargest(1);
    
    const int id = cluster.getLargestClusterId();
    const int size0 = cluster.pointCount();

    // Add points and expand cluster
    cluster.addPoints(points);
    cluster.expandSingleCluster(id);

    // Allocate vector for associated points
    std::vector<int> inliers;
    inliers.reserve(points.size());

    // Iterate through labels and check if any points were assigned to the cluster
    const std::vector<int>& labels = cluster.labels();
    for (int i = size0; i < labels.size(); ++i){
        if (labels[i] == 1)
            inliers.push_back(i);
    }
    
    return inliers;
}