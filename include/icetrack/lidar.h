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

#include "icetrack/common.h"
#include "icetrack/container.h"

#include "icetrack/factors/AltitudeFactor.h"
#include <gtsam/navigation/AttitudeFactor.h>

class LidarHandle{
public: 
    LidarHandle();
    LidarHandle(ros::NodeHandle nh);

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
    ros::NodeHandle nh_;

    Pose3 bTl_;

    std::shared_ptr<PointCloudBuffer> point_buffer_;

    bool init_ = false;
    double ts_head_ = 0.0; // Track the latest point stamp to assert chronological order on insertion

    // From plane measurements (given in body-frame)
    double z_ = 0.0;    // Distance from ice sheet
    Unit3 bZ_;          // Body frame normal vector of ice sheet

    double measurement_interval_;
    double measurement_sigma_;
    double min_x_; // Minimum x-value for acceptance of a point
    double min_inlier_count_;

    double point_interval_;    // Temporal distance between measurements (We use half the interval because of dual return mode)
    double buffer_period_ ;       // Keep track of only most recent points

    double ransac_threshold_;
    double ransac_prob_;
    pcl::SACSegmentation<pcl::PointXYZI> seg_;

    bool segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
};