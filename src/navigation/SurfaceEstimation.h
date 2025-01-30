#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include <open3d/geometry/PointCloud.h>

#include "navigation/navigation.h"
#include "navigation/LidarPointBuffer.h"
#include "navigation/factors/AltitudeFactor.h"

#include "utils/ros_params.h"
#include "utils/calibration.h"
#include "utils/StampedRingBuffer.h"

using namespace gtsam;

class SurfaceEstimation{
public: 
    SurfaceEstimation(const ros::NodeHandle& nh, const LidarPointBuffer& point_buffer);

    bool estimateSurface(double ts);
    double getSurfaceDistance() const{ return distance_; }
    Unit3 getSurfaceNormal() const{ return normal_; }

    AltitudeFactor getAltitudeFactor(Key pose_key) const;
    Pose3AttitudeFactor getAttitudeFactor(Key pose_key) const;

private:
    Pose3 bTl_; // Extrinsic matrix (Lidar->Imu)

    const LidarPointBuffer& point_buffer_;

    // Plane parameters
    double distance_;
    Unit3 normal_;

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