#include "frontend/Floe.h"

std::vector<int> Floe::findOutliers(){
    // Run clustering
    RasterizedCluster cluster(cloud_->points_);
    cluster.runClustering();

    // Keep only largest cluster
    cluster.keepLargest(1);
    return cluster.getNoise();
}