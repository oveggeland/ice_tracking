#include "frontend/FloeManager.h"

// Constructor
FloeManager::FloeManager(const ros::NodeHandle& nh, const PoseGraph& pg, FrameBuffer& fb) : pg_(pg), fb_(fb) {
    background_ = Floe(0);
};


void FloeManager::processFrame(const CloudFrame& frame){
    const int frame_size = frame.size(); 
    if (frame_size == 0)
        return;

    // Floe labels
    const std::vector<int>& floe_labels = frame.floeLabels();

    // Transform points
    open3d::geometry::PointCloud cloud(frame.local());
    const auto& bTw = frame.transform();
    cloud.Transform(bTw);
    const std::vector<Eigen::Vector3d>& points = cloud.points_;

    // Iterate over all points in frame
    for (int i = 0; i < frame_size; ++i){
        const int& label = floe_labels[i];

        if (label == 0)
            background_.addPoint(points[i], frame.idx(), i);
        else
            floes_[label].addPoint(points[i], frame.idx(), i);
    }
}

/*
Iterate over all frames and refine the floes
*/
void FloeManager::updateFloes(){
    // Clear for rebuild
    background_.clear();
    clearFloes();

    // Allocate memory for background points (floes are allocated on creation)
    const int point_count = fb_.pointCount(); // Upper bound
    background_.reserve(point_count);

    // Refine based on each frame in the buffer
    for (auto frame_it = fb_.begin(); frame_it != fb_.end(); ++frame_it){
        processFrame(*frame_it);
    }

    // Maintain
    for (auto it = floes_.begin(); it != floes_.end(); ) {
        Floe& floe = it->second;

        // Reassign outliers to "background"
        std::vector<int> outliers = floe.findOutliers();
        if (outliers.size() > 0)
            reassignPoints(floe, background_, outliers);

        // Erase
        if (it->second.size() < min_floe_size_) {
            reassignPoints(it->second, background_);
            it = floes_.erase(it);
        } 
        else
            ++it;
    }
}

int FloeManager::findFloeMatch(const Eigen::Vector3d& point){
    for (auto& [floe_id, floe] : floes_) {
        if (floe.isCompatible(point))
            return floe_id; // Use first 
    }
    return 0;
}

// Expand floes with points from background
void FloeManager::expandFloes(){
    // for (auto& floe: floes_){
    //     std::vector<int> inliers = floe_.findInliers(*background_.getCloud());
    //     reassignPoints(background_, floe, inliers);
    // }
}

// Reassign a single point from source to target
// Does not remove the point from source!
void FloeManager::reassignPoint(const Floe& source, Floe& target, const int idx){
    fb_.setFloeLabel(source.frame_id_[idx], source.frame_idx_[idx], target.id());
    target.copyFrom(source, idx);
}

/*
Reassigns a points based from source to target, based on the indices of source.
*/
void FloeManager::reassignPoints(Floe& source, Floe& target, const std::vector<int>& indices){
    target.reserveAdditional(indices.size());
    
    for (const int idx: indices) {
        reassignPoint(source, target, idx);
    }

    source.removeByIndex(indices);
}

/*
Reassign all points from source to target
*/
void FloeManager::reassignPoints(Floe& source, Floe& target){
    const int n_points = source.size();
    target.reserveAdditional(n_points);
    
    for (int idx = 0; idx < n_points; ++idx) {
        reassignPoint(source, target, idx);
    }

    source.clear();
}




#include <chrono>
#include <iostream>

void FloeManager::discoverFloes(){
    auto cloud = background_.getCloud()->points_;
    if (cloud.size() < 1)
        return;
    
    ClusterRaster raster(cloud, 1.0);
    std::vector<std::vector<int>> clusters = raster.getClusters();


    for (const auto& cluster: clusters){
        if (cluster.size() > min_floe_size_){
            // Okay, lets make a new flow with id and pre-determined capacity
            Floe new_floe(floe_id_counter_++, cluster.size());

            // Removes points from background and puts them in new_floe
            reassignPoints(background_, new_floe, cluster);

            // Add floe to buffer
            floes_[new_floe.id()] = new_floe;

            break; // Only allow one new flow every time
        }
    }

    // if (pointCount() > 100000)
    //     visualizeFloes();
}

int FloeManager::pointCount() const {
    int cnt = 0;
    for (auto& [floe_id, floe] : floes_) {
        cnt += floe.size();
    }
    return cnt;
}

void FloeManager::clearFloes() {
    for (auto& [floe_id, floe] : floes_) {
        floe.clear();
    }
}


void FloeManager::visualizeFloes() {
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> clouds_to_visualize;

    std::mt19937 gen(std::random_device{}());  // Random generator
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);  // Range for color components

    // Iterate over all floes and assign random colors
    for (auto& [floe_id, floe] : floes_) {
        // Assign random color to the floe
        floe.setColor(Eigen::Vector3d(dist(gen), dist(gen), dist(gen)));

        // Add the cloud to the list of clouds to visualize
        clouds_to_visualize.push_back(floe.getCloud());
    }

    // Visualize all the clouds
    open3d::visualization::DrawGeometries(clouds_to_visualize, "Floe Visualization");
}
