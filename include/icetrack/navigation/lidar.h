#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>  // For SACSegmentation

#include <yaml-cpp/yaml.h>

#include "icetrack/navigation/altitudeFactor.h"
#include "icetrack/navigation/common.h"

#include "icetrack/mapping/container.h"

#include <gtsam/navigation/AttitudeFactor.h>

class LidarHandle{
public: 
    LidarHandle();

    void init(double ts);
    bool isInit();
    
    double getAltitude();

    bool generatePlane(double ts);

    void addFrame(sensor_msgs::PointCloud2::ConstPtr msg);
    boost::shared_ptr<gtsam::NonlinearFactor> getAltitudeFactor(Key key);
    boost::shared_ptr<gtsam::NonlinearFactor> getAttitudeFactor(Key key);

private:
    Pose3 bTl_;

    PointCloudBuffer point_buffer_;

    bool init_ = false;

    // From plane measurements (given in body-frame)
    double z_ = 0.0;    // Distance from ice sheet
    Unit3 bZ_;          // Body frame normal vector of ice sheet

    double measurement_interval_ = 0.2;
    double measurement_sigma_ =  1;
    double min_x_distance_ = 5;
    double min_inlier_count_ = 100;

    double point_interval_ = 5.0e-6;    // Temporal distance between measurements (We use half the interval because of dual return mode)
    double cloud_interval_ = 10.0;      // Size of sliding window point cloud

    pcl::SACSegmentation<pcl::PointXYZI> seg_;

    bool segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};