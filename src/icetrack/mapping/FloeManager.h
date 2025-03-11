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
    void rebuildFloes();
    void expandFloes();
    void mergeFloes();

    void discoverFloes(); 

    // Accessors
    int floeCount() const { return floes_.size(); }
    void visualizeFloes();

private:
    // Background ice
    Floe background_; 

    // Floe tracking
    int floe_id_counter_ = 1;
    std::map<int, Floe> floes_;     // Maps a floe id (int) to a Floe object

    double min_floe_area_ = 50; // Square meters
    int min_floe_size_ = 3000;  // Number of points
    double floe_intersection_threshold_ = 10.0;

    // Resources
    const PoseGraph& pg_;       // For state estimates
    FrameBuffer& fb_;     // For point access


    void reassignPoints(Floe& source, Floe& target); 
    void reassignPoints(Floe& source, Floe& target, const std::vector<int>& indices);


    void rebuildFromFrame(const CloudFrame& frame); // refine floes and map based on input frame

    // Try to merge floe0 and floe1
    bool mergeFloePair(Floe& floe0, Floe& floe1);

    void clearFloes();                                      // Reset every floe in floes_ (clears point cloud)
};