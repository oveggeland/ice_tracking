#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>  // For SACSegmentation

#include <yaml-cpp/yaml.h>

#include "icetrack/navigation/altitudeFactor.h"
#include "icetrack/navigation/common.h"

class LidarHandle{
public: 
    LidarHandle();

    void init(sensor_msgs::PointCloud2::ConstPtr msg);
    bool isInit();
    
    double getAltitude();
    boost::shared_ptr<gtsam::NonlinearFactor> getCorrectionFactor(sensor_msgs::PointCloud2::ConstPtr msg, Key key, bool &success);

private:
    Pose3 bTl_;

    bool init_ = false;
    double ts_ = 0.0;
    double z_ = 0.0;

    double measurement_interval_;
    double measurement_sigma_;
    double min_x_distance_;
    int min_inlier_count_;

    pcl::SACSegmentation<pcl::PointXYZ> seg_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr msgToCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector4 &plane_coeffs);
};