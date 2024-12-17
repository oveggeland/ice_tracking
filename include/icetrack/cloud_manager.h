#pragma once

#include <gtsam/geometry/Pose3.h>
#include <pcl/io/ply_io.h>

#include "icetrack/container.h"
#include "icetrack/lidar.h"

class CloudManager{
public:
    CloudManager(){};
    CloudManager(std::shared_ptr<LidarHandle> lidar);

    void newPose(double ts, Pose3 T);

private:
    PointCloudBuffer cloud_;
    std::shared_ptr<PointCloudBuffer> point_buffer_;

    double window_size_ = 20.0; // Size of sliding window to use for analysis
    double z_mean_, z_var_;

    // Keep track of previous pose
    double ts_prev_;
    Pose3 pose_prev_;

    double x0_ = 0.0;
    double y0_ = 0.0;

    void saveCloud();

    void analyseWindow();
    void calculateMoments();
};