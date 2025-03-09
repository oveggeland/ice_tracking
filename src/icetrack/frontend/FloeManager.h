#pragma once

#include <ros/ros.h>

#include "backend/PoseGraph.h"

#include "frontend/Floe.h"
#include "frontend/FrameBuffer.h"

#include "frontend/RasterizedCluster.h"

class FloeManager {
public:
    FloeManager(const ros::NodeHandle& nh, const PoseGraph& pg, FrameBuffer& fb);

    // Top level functionality (in order of execution)
    void updateFloes();
    void expandFloes();
    void discoverFloes(); 

    // Accessors
    int getFloeCount() const { return floes_.size(); }
    void visualizeFloes();

private:
    // Background ice
    Floe background_; 

    // Floe tracking
    int floe_id_counter_ = 0;
    std::map<int, Floe> floes_;     // Maps a floe id (int) to a Floe object

    int min_floe_size_ = 3000;

    // Resources
    const PoseGraph& pg_;       // For state estimates
    FrameBuffer& fb_;     // For point access

    // Helpers
    int findFloeMatch(const Eigen::Vector3d& point);

    void reassignPoint(const Floe& source, Floe& target, const int idx);
    void reassignPoints(Floe& source, Floe& target); 
    void reassignPoints(Floe& source, Floe& target, const std::vector<int>& indices);

    int pointCount() const;

    void processFrame(const CloudFrame& frame); // refine floes and map based on input frame

    void clearFloes();                                      // Reset every floe in floes_ (clears point cloud)
    void reserveMemoryForFloes(const int n_points);         // Reserve enough space in floe for n points
};