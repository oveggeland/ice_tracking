/*
IceTrack is the main entry point of the library. 
Here, callbacks from different sensors are buffered and processed in chronological order according to their header timestamp.
*/

#pragma once

#include "icetrack/navigation/PoseEstimator.h"
#include "icetrack/CloudManager.h"
#include "icetrack/system/SensorSystem.h"

#include "icetrack/Diagnostics.h"
#include "icetrack/utils/utils.h"

class IceTrack{
public:
    IceTrack();
    IceTrack(ros::NodeHandle nh);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void shipCallback(const icetrack::ShipNavigation::ConstPtr& msg);
private:
    // Time control
    double t_head_ = 0.0;       // Last processed message
    double t_safe_ = 0.0;       // Earliest time allowed for processing a message
    double safe_delay_ = 0.5;   // Delay between latest incoming message timestamp and t_safe_

    // Saving incoming measurements in buffer and process in chronological time after safe_delay has expired
    std::map<double, std::function<void()>> callback_buffer_;
    void addCallback(double ts, std::function<void()> cb);
    void checkCallbackBuffer();

    // Submodules
    std::shared_ptr<SensorSystem> system_;
    PoseEstimator pose_estimator_;
    CloudManager cloud_manager_;
    Diagnostics diag_;
    
    // Safe callback functions
    void imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void pclSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void shipSafeCallback(const icetrack::ShipNavigation::ConstPtr& msg);
};