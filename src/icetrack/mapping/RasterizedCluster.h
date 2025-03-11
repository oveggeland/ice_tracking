#pragma once

#include <ros/ros.h>

#include <map>
#include <random>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "Raster.h"

class RasterizedCluster{
public:
    // Constructors
    RasterizedCluster(const Raster& raster)
         : raster_(raster) {}

    RasterizedCluster(const std::vector<Eigen::Vector3d>& points, const double grid_size=1.0)
         : raster_(points, grid_size) {}


    void addPoints(const std::vector<Eigen::Vector3d>& points);


    void runClustering();
    std::unordered_map<int, std::vector<int>> getClusters() { return clusters_; }

    int clusterCount() const { return clusters_.size(); }
    int pointCount() const { return labels_.size(); }
    
    // Used for floe maintanance
    void keepLargest(const int n_clusters);
    void keepBySize(const int min_points) {}
    std::vector<int> getNoise();


    void expandSingleCluster(const int cluster_id);

    void visualizeClusters();

    std::vector<int> getBiggestCluster() const;
    int getLargestClusterId() const;

    double getClusterArea(const int cluster_id) const {
        const auto cluster_it = super_clusters_.find(cluster_id);
        if (cluster_it == super_clusters_.end())
            return 0.0; 
        return cluster_it->second.size() * raster_.cellArea();
    }

    // Accessors
    const std::vector<int>& superLabels() const { return super_labels_; }
    const std::vector<int>& labels() const { return labels_; }

    const std::unordered_map<int, std::vector<int>>& superClusters() const { return super_clusters_; }
    const std::unordered_map<int, std::vector<int>>& clusters() const { return clusters_; }

private:
    Raster raster_;

    int cluster_id_ = 1;
    size_t eps_ = 2;
    size_t min_points_ = 200;

    // Raster scale labels and clusters
    std::vector<int> super_labels_;
    std::unordered_map<int, std::vector<int>> super_clusters_;

    // Full scale labels and clusters
    std::vector<int> labels_;
    std::unordered_map<int, std::vector<int>> clusters_;

    void expandCluster(const int idx);
    void retrace();

    void removeCluster(const int idx);
};