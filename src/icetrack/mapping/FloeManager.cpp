#include "frontend/FloeManager.h"

// Constructor
FloeManager::FloeManager(const ros::NodeHandle& nh, const PoseGraph& pg, FrameBuffer& fb) : pg_(pg), fb_(fb) {
    // Initialize background floe
    background_ = Floe(0);
    background_.setColor({0, 0, 0});
};


void FloeManager::rebuildFromFrame(const CloudFrame& frame){
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
void FloeManager::rebuildFloes(){
    // Clear for rebuild
    background_.clear();
    clearFloes();

    // Allocate memory for background points (floes are pre-allocated on creation)
    const int point_count = fb_.pointCount(); // Upper bound
    background_.reserve(point_count);

    // Rebuild based on each frame
    for (auto frame_it = fb_.begin(); frame_it != fb_.end(); ++frame_it){
        rebuildFromFrame(*frame_it);
    }

    // Maintain
    for (auto it = floes_.begin(); it != floes_.end(); ) {
        Floe& floe = it->second;

        // Reassign outliers to "background"
        std::vector<int> outliers = floe.findOutliers();
        if (outliers.size() > 0){
            reassignPoints(floe, background_, outliers);
            ROS_DEBUG_STREAM("Removed " << outliers.size() << " outliers from floe " << it->first);
        }

        // Erase
        if (floe.size() < min_floe_size_) {
            ROS_DEBUG_STREAM("Floe " << floe.id() << " with size " << floe.size() << " will be deleted");
            reassignPoints(it->second, background_);
            it = floes_.erase(it);
        } 
        else
            ++it;
    }
}

// Expand floes with points from background
void FloeManager::expandFloes(){
    for (auto& [floe_id, floe]: floes_){
        std::vector<int> inliers = floe.associatePoints(background_.getCloud()->points_);
        reassignPoints(background_, floe, inliers);

        ROS_DEBUG_STREAM("Expanded floe " << floe_id << " with " << inliers.size() << " points");
    }
}

// Check and see if floe0 and floe1 should be merged
bool FloeManager::mergeFloePair(Floe& floe0, Floe& floe1){
    if (floe0.intersection(floe1) > floe_intersection_threshold_){
        ROS_WARN_STREAM("Merge floe " << floe0.id() << " and " << floe1.id());

        // Merge the smaller into the bigger
        if (floe0.getArea() > floe1.getArea()){
            reassignPoints(floe1, floe0);
            floes_.erase(floe1.id());
        }
        else{
            reassignPoints(floe0, floe1);
            floes_.erase(floe0.id());
        }

        return true;
    }
    return false;
}

// Try all floe-to-floe combinations and see if they should be merged
void FloeManager::mergeFloes() {
    std::vector<std::pair<int, int>> floe_pairs;

    // Only compare each pair once
    for (auto floe0_it = floes_.begin(); floe0_it != floes_.end(); ++floe0_it) {
        for (auto floe1_it = std::next(floe0_it); floe1_it != floes_.end(); ++floe1_it) {
            floe_pairs.emplace_back(floe0_it->first, floe1_it->first);
        }
    }

    // Process the merging after gathering all pairs
    for (const auto& pair : floe_pairs) {
        auto floe0_it = floes_.find(pair.first);
        auto floe1_it = floes_.find(pair.second);

        if (floe0_it != floes_.end() && floe1_it != floes_.end()){
            mergeFloePair(floe0_it->second, floe1_it->second);
        }
    }
}

/*
Reassign all points from source to target
*/
void FloeManager::reassignPoints(Floe& source, Floe& target){
    target.copyFrom(source);

    const int target_id = target.id(); 
    const int n_points = source.size();
    for (int idx = 0; idx < n_points; ++idx){
        fb_.setFloeLabel(source.getFrameId(idx), source.getFrameIdx(idx), target_id);
    }

    source.clear();
}

/*
Reassign points from source to target based on a index list
*/
void FloeManager::reassignPoints(Floe& source, Floe& target, const std::vector<int>& indices){
    target.copyFrom(source, indices);

    const int target_id = target.id(); 
    for (const int idx: indices) {
        fb_.setFloeLabel(source.getFrameId(idx), source.getFrameIdx(idx), target_id);
    }
    source.removeByIndex(indices);
}

void FloeManager::discoverFloes(){
    if (background_.size() < min_floe_size_)
        return;
    
    // Cluster background points
    RasterizedCluster raster(background_.getCloud()->points_);
    raster.runClustering();

    // Iterate through clusters
    const auto& clusters = raster.clusters();
    for (auto it = clusters.begin(); it != clusters.end(); ++it){
        // Fetch cluster data
        const int cluster_id = it->first;
        const std::vector<int>& cluster = it->second;

        if (raster.getClusterArea(cluster_id) > min_floe_area_ &&
            cluster.size() > min_floe_size_){

            // Woho, new floe!
            Floe new_floe(floe_id_counter_++, cluster.size());

            // Removes points from background and puts them in new_floe
            reassignPoints(background_, new_floe, cluster);

            // Add floe to buffer
            floes_[new_floe.id()] = new_floe;
            break; // For now, only allow one new floe each round
        }
    }
}


void FloeManager::clearFloes() {
    for (auto& [floe_id, floe] : floes_) {
        floe.clear();
    }
}


void FloeManager::visualizeFloes(){
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> clouds_to_visualize;
    clouds_to_visualize.push_back(background_.getCloud());

    for (auto& [floe_id, floe] : floes_) {
        clouds_to_visualize.push_back(floe.getCloud());
    }

    open3d::visualization::DrawGeometries(clouds_to_visualize, "Floe Visualization");
}
