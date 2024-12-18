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

#include "icetrack/altitudeFactor.h"
#include "icetrack/common.h"

#include "icetrack/container.h"

#include <gtsam/navigation/AttitudeFactor.h>

class LidarHandle{
public: 
    LidarHandle();

    void init(double ts);
    bool isInit();
    
    double getPointInterval(){return point_interval_;}

    std::shared_ptr<PointCloudBuffer> getSharedBufferPointer(){
        return point_buffer_;
    }

    double getAltitude();
    bool generatePlane(double ts);

    void addFrame(sensor_msgs::PointCloud2::ConstPtr msg);
    boost::shared_ptr<gtsam::NonlinearFactor> getAltitudeFactor(Key key);
    boost::shared_ptr<gtsam::NonlinearFactor> getAttitudeFactor(Key key);

private:
    Pose3 bTl_;

    std::shared_ptr<PointCloudBuffer> point_buffer_;

    bool init_ = false;
    double ts_head_ = 0.0; // Track the latest point stamp to assert chronological order on insertion

    // From plane measurements (given in body-frame)
    double z_ = 0.0;    // Distance from ice sheet
    Unit3 bZ_;          // Body frame normal vector of ice sheet

    double measurement_interval_ = 0.2;
    double measurement_sigma_ =  1;
    double min_x_ = 10; // Minimum x-value for acceptance of a point
    double min_inlier_count_ = 100;

    double point_interval_ = 5.0e-6;    // Temporal distance between measurements (We use half the interval because of dual return mode)
    double buffer_period_ = 2.0;       // Keep track of only most recent points

    pcl::SACSegmentation<pcl::PointXYZI> seg_;

    bool segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};