#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>  // For SACSegmentation

#include <yaml-cpp/yaml.h>

#include "icetrack/common.h"
#include "icetrack/StampedRingBuffer.h"
#include "icetrack/SensorSystem.h"

#include "icetrack/factors/AltitudeFactor.h"
#include <gtsam/navigation/AttitudeFactor.h>

class SurfaceEstimator{
public: 
    SurfaceEstimator();
    SurfaceEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system);

    void init(double ts);
    bool isInit();

    double getAltitude();
    bool generatePlane(double ts);

    void addFrame(sensor_msgs::PointCloud2::ConstPtr msg);
    boost::shared_ptr<gtsam::NonlinearFactor> getAltitudeFactor(Key key);
    boost::shared_ptr<gtsam::NonlinearFactor> getAttitudeFactor(Key key);

private:
    Pose3 bTl_;

    std::shared_ptr<StampedRingBuffer<PointXYZI>> point_buffer_;

    bool init_ = false;
    double ts_head_ = 0.0; // Track the latest point stamp to assert chronological order on insertion

    // From plane measurements (given in body-frame)
    double z_ = 0.0;    // Distance from ice sheet
    Unit3 bZ_;          // Body frame normal vector of ice sheet

    double measurement_interval_;
    double measurement_sigma_;
    double min_intensity_;
    double min_dist_square_, max_dist_square_; // Square range for initial outlier rejection
    double min_inlier_count_;

    double ransac_threshold_;
    double ransac_prob_;
    pcl::SACSegmentation<pcl::PointXYZI> seg_;

    bool segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};