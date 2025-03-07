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
        if (it->second.size() < min_floe_size_) {
            ROS_INFO_STREAM("Delete floe: " << it->first);
            reassignPoints(it->second, background_);
            it = floes_.erase(it);
        } 
        else {
            it->second.buildSearchTree();
            ++it;
        }
    }
}

int FloeManager::assignToFloe(const Eigen::Vector3d& point){
    for (auto& [floe_id, floe] : floes_) {
        if (floe_id == 0)
            continue; // Skip background

        if (floe.isCompatible(point))
            return floe_id; // Use first 
    }
    return 0;
}

// Expand floes with points from background (including newly added frame)
void FloeManager::expandFloes(){
    ROS_WARN("expandFloes() not implemented");
    return;
    // Extract background points
    const std::vector<Eigen::Vector3d>& points = background_.getCloud()->points_;
    const int n_points = points.size();
    
    // Allocate vector for floe labels
    std::vector<int> floe_label;
    floe_label.reserve(n_points);

    // Iterate and assign a floe label
    for (int i = 0; i < n_points; ++i){
        floe_label.push_back(assignToFloe(points[i]));
    }

    // 
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




void FloeManager::discoverFloes(){
    // Down-sample and trace
    auto cloud = background_.getCloud();
    auto min_bound = cloud->GetMinBound();
    auto max_bound = cloud->GetMaxBound();
    auto [cloud_ds, trace, inliers] = cloud->VoxelDownSampleAndTrace(1.0, min_bound, max_bound);

    // Cluster
    std::vector<int> labels = cloud_ds->ClusterDBSCAN(5.0, 50);

    // Convert to a list of per-label indices
    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < labels.size(); ++i){
        const int idx = labels[i]+1;
        if (idx >= clusters.size())
            clusters.resize(idx+1);
        clusters[idx].push_back(i);
    }

    // Allocation cluster trace vector
    std::vector<int> cluster_trace;
    cluster_trace.reserve(background_.size()); // Upper limit reservation

    // Iterate through non-background clusters
    for (int i = 1; i < clusters.size(); ++i){
        auto& cluster = clusters[i];

        // Get cluster trace
        for (const int voxel_idx: cluster){
            std::vector<int>& voxel_points = inliers[voxel_idx];
            cluster_trace.insert(cluster_trace.end(), voxel_points.begin(), voxel_points.end());
        }
    
        // Check if cluster is big enough
        if (cluster_trace.size() > min_floe_size_){
            ROS_INFO_STREAM("Add floe: " << floe_id_counter_ << " of size: " << cluster_trace.size());
            // Okay, lets make a new flow with id and pre-determined capacity
            Floe new_floe(floe_id_counter_++, cluster_trace.size());

            // Removes points from background and puts them in new_floe
            reassignPoints(background_, new_floe, cluster_trace);

            // Add floe to buffer
            floes_[new_floe.id()] = new_floe;

            break; // Only allow one new flow every time
        }

        cluster_trace.clear();
    }

    if (pointCount() > 100000)
        visualizeFloes();
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
