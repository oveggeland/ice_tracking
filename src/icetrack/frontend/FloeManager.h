#pragma once

#include <ros/ros.h>

#include "backend/PoseGraph.h"

#include "frontend/Floe.h"
#include "frontend/FrameBuffer.h"

class FloeManager {
public:
    FloeManager(const ros::NodeHandle& nh, const PoseGraph& pg, FrameBuffer& fb);

    // Top level functionality
    void refineFloes();
    void discoverNewFloes(); 

    // Accessors
    int getFloeCount() const { return floes_.size(); }

private:
    // Floe tracking (first element is always "background ice")
    int max_floes_;
    int floe_id_counter_;
    std::map<int, Floe> floes_;     // Maps a floe id (int) to a Floe object

    // Resources
    const PoseGraph& pg_;       // For state estimates
    FrameBuffer& fb_;     // For point access

    // Helpers
    void reassignPoints(Floe source, Floe& target, const std::vector<int>& indices);

    void processFrame(const CloudFrame& frame); // refine floes and map based on input frame

    void clearAllFloes();                                 // Reset every floe in floes_ (clears point cloud)
    void reserveMemoryForFloes(const int n_points);         // Reserve enough space in floe for n points

    void visualizeFloes();
};