#pragma once

#include <sensor_msgs/Imu.h>

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "utils/ros_params.h"
#include "navigation/navigation.h"

class ImuIntegration{
    public:
        // Constructor
        ImuIntegration(ros::NodeHandle nh);

        // Interface
        void newMeasurement(const sensor_msgs::Imu::ConstPtr& msg);

        void finishIntegration(double ts);
        void resetIntegration(double ts, imuBias::ConstantBias bias);

        NavState predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias) const;
        bool timeOut() const { 
            bool ret =  (ts_head_ - ts_tail_) > timeout_interval_; 
            if (ret){
                ROS_INFO_STREAM(std::fixed << ts_head_ << ", " << ts_tail_ << ", " << timeout_interval_);
            }
            return ret;
        };

        
        CombinedImuFactor getIntegrationFactor(int state_idx);

        Rot3 estimateAttitude() const;
        Pose3AttitudeFactor getAttitudeFactor(Key key) const;

    private:
        // Control
        int seq_ = 0;
        double ts_head_;
        double ts_tail_; 
        double timeout_interval_;

        // Measurements
        Vector3 acc_;
        Vector3 rate_;
        void setAcc(const sensor_msgs::Imu::ConstPtr& msg);
        void setRate(const sensor_msgs::Imu::ConstPtr& msg);

        // Preintegration class
        std::shared_ptr<PreintegrationType> preintegration_;
        boost::shared_ptr<PreintegrationCombinedParams> getPreintegrationParams() const;

        // Tunable parameters
        double gravity_norm_;

        double accel_noise_sigma_;
        double gyro_noise_sigma_;
        double accel_bias_rw_sigma_;
        double gyro_bias_rw_sigma_;

        double imu_attitude_sigma_;
};