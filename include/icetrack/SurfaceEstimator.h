#pragma once

#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>

#include "icetrack/SensorSystem.h"
#include "icetrack/StampedRingBuffer.h"
#include "icetrack/factors/AltitudeFactor.h"


class SurfaceEstimator{
public: 
    SurfaceEstimator();
    SurfaceEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system);

    bool estimateSurface(double ts);

    double getSurfaceDistance();

    boost::shared_ptr<gtsam::NonlinearFactor> getAltitudeFactor(gtsam::Key key);
    boost::shared_ptr<gtsam::NonlinearFactor> getAttitudeFactor(gtsam::Key key);

private:
    Pose3 bTl_; // Extrinsic matrix (Lidar->Imu)

    std::shared_ptr<StampedRingBuffer<RawLidarPoint>> point_buffer_; // Lidar point buffer (Incoming points in lidar frame)

    double frame_interval_;         // Frame size used for plane fitting

    double surface_dist_;
    gtsam::Unit3 surface_normal_;

    double ransac_threshold_;
    int ransac_sample_size_;
    int ransac_inlier_count_;
    int ransac_iterations_;

    double sigma_altitude_;
    double sigma_attitude_;
};