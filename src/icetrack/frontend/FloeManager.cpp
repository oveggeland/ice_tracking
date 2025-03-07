#include "frontend/FloeManager.h"

// Constructor
FloeManager::FloeManager(const ros::NodeHandle& nh, const PoseGraph& pg, FrameBuffer& fb) : pg_(pg), fb_(fb) {
    max_floes_ = 4;
    floe_id_counter_ = 1;
    floes_[0] = Floe(0);
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
        floes_[label].addPoint(points[i], frame.idx(), i);
    }
}

/*
Iterate over all frames and refine the floes
*/
void FloeManager::refineFloes(){
    // Reset floes
    clearAllFloes();

    // Allocate enough memory in all floes
    const int point_count = fb_.pointCount(); // Upper bound (if all points are assigned to a single floe)
    reserveMemoryForFloes(point_count);

    // Refine based on each lidar frame in the buffer
    for (auto frame_it = fb_.begin(); frame_it != fb_.end(); ++frame_it){
        processFrame(*frame_it);
    }

    // TODO: Remove old/small/empty floes


    // if (point_count > 100000)
    //     visualizeFloes();
}


/*
Reassigns a set of points from one Floe object to another. Indices represent the point idx of source. 
*/
void FloeManager::reassignPoints(const Floe source, Floe& target, const std::vector<int>& indices){
    for (const int idx: indices) {
        // Reassign label
        // fb_.setFloeLabel(source.frame_id_[idx], source.frame_idx_[idx], target.id());
        
        // Copy data
        target.copyFrom(source, idx);
    }
}



void FloeManager::discoverNewFloes(){
    int min_floe_size_ = 5000;

    // Get background floe
    Floe& background = floes_.at(0);
    if (background.size() < min_floe_size_)
        return;

    // Down-sample
    auto cloud = background.getCloud();
    auto min_bound = cloud->GetMinBound();
    auto max_bound = cloud->GetMaxBound();
    auto [cloud_ds, trace, inliers] = background.getCloud()->VoxelDownSampleAndTrace(1.0, min_bound, max_bound);

    // Cluster
    std::vector<int> labels = cloud_ds->ClusterDBSCAN(5.0, 50);

    // Convert to a list per-label indices
    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < labels.size(); ++i){
        const int idx = labels[i]+1;
        if (idx >= clusters.size())
            clusters.resize(idx+1);
        clusters[idx].push_back(i);
    }


    Floe new_background;
    int new_floe_count = 0;


    std::vector<int> cluster_trace;
    cluster_trace.reserve(background.size());
    for (int i = 0; i < clusters.size(); ++i){
        auto& cluster = clusters[i];
        for (auto voxel_idx: cluster){
            std::vector<int>& voxel_points = inliers[voxel_idx];
            cluster_trace.insert(cluster_trace.end(), voxel_points.begin(), voxel_points.end());
        }
        
        if (i == 0){
            new_background = Floe(0, cluster_trace.size());
            reassignPoints(background, new_background, cluster_trace);
        }
        else if (cluster_trace.size() > min_floe_size_){
            ROS_INFO_STREAM("Found a big non-background cluster of size: " << cluster_trace.size());

            // Okay, lets make a new flow with id and pre-determined capacity
            Floe new_floe(++floe_id_counter_, cluster_trace.size());

            // Removes points from background and puts them in new_floe
            reassignPoints(background, new_floe, cluster_trace);

            // Add floe to buffer
            floes_[new_floe.id()] = new_floe;

            new_floe_count++;
            //break; // We just messed up the trace in reassignPoints...
        }

        cluster_trace.clear();
    }

    background = new_background;

    if (new_floe_count > 1 && fb_.pointCount() > 100000)
        visualizeFloes();
}



void FloeManager::clearAllFloes() {
    for (auto& [floe_id, floe] : floes_) {
        floe.clear();
    }
}

void FloeManager::reserveMemoryForFloes(const int n_points) {
    for (auto& [floe_id, floe] : floes_) {
        floe.reserve(n_points);
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
