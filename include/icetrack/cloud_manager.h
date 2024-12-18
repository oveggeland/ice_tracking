#pragma once

#include <gtsam/geometry/Pose3.h>
#include <pcl/io/ply_io.h>

#include "icetrack/container.h"
#include "icetrack/lidar.h"

class CloudManager{
public:
    // Constructurs
    CloudManager(){};
    CloudManager(std::shared_ptr<LidarHandle> lidar);

    // Main entry from IceTrack
    void newPose(double ts, Pose3 T);

private:
    // Cloud buffers
    std::shared_ptr<PointCloudBuffer> point_buffer_;    // Incoming body-frame points   (Owned by LidarHandle)
    PointCloudBuffer cloud_;                            // World-frame points           (Owned by CloudManager)
    // Keep track of previous pose
    double ts_prev_ = 0.0;
    Pose3 pose_prev_;

    // Keep track of offset
    double x0_ = 0.0;
    double y0_ = 0.0;
    Pose3 shiftPose(Pose3 pose);

    // Initializing
    bool init_ = false;
    bool isInit();
    void initialize(double t0, Pose3 pose0);

    // Cloud management and processing
    double window_size_ = 10.0; // Size of sliding window to use for analysis
    void saveCloud();
    void analyseWindow();
    void calculateMoments();
};