#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include <open3d/geometry/PointCloud.h>

#include "icetrack/utils/ros_params.h"
#include "icetrack/utils/calibration.h"
#include "icetrack/utils/StampedRingBuffer.h"
#include "icetrack/navigation/factors/AltitudeFactor.h"

using namespace gtsam;

class SurfaceEstimation{
public: 
    SurfaceEstimation(ros::NodeHandle nh);

    // Interface
    void addLidarFrame(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool estimateSurface(double ts);
    double getSurfaceDistance() { return surface_distance_; }
    Unit3 getSurfaceNormal() { return surface_normal_; }

    AltitudeFactor getAltitudeFactor(Key pose_key);
    Pose3AttitudeFactor getAttitudeFactor(Key pose_key);

private:
    Pose3 bTl_; // Extrinsic matrix (Lidar->Imu)

    StampedRingBuffer<PointXYZT> point_buffer_;

    // Lidar parameters
    double ts_head_ = 0.0;
    double point_interval_;

    double min_intensity_;
    double min_dist_squared_;
    double max_dist_squared_;

    // Plane parameters
    double surface_distance_;
    Unit3 surface_normal_;

    // Ransac
    double ransac_frame_size_;
    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_inlier_count_;
    int ransac_iterations_;

    // Factors
    double sigma_altitude_;
    double sigma_attitude_;
};