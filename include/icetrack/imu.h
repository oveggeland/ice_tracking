#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include "icetrack/common.h"

Vector3 getAcc(const sensor_msgs::Imu::ConstPtr& msg);
Vector3 getRate(const sensor_msgs::Imu::ConstPtr& msg);

class ImuHandle{
    public:
        // Constructor
        ImuHandle();
        ImuHandle(ros::NodeHandle nh);

        // Integration
        void integrate(const sensor_msgs::Imu::ConstPtr& msg);
        CombinedImuFactor finishIntegration(double ts_correction, int correction_count);
        void resetIntegration(double ts, imuBias::ConstantBias bias);

        // Tools
        NavState predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias);
        Rot3 getPriorRot();
        boost::shared_ptr<gtsam::NonlinearFactor> getAttitudeFactor(Key key);
        Unit3 getNz();

        // Control
        void init(const sensor_msgs::Imu::ConstPtr& msg);
        bool isInit(){return is_init_;};

    private:
        ros::NodeHandle nh_;
        
        // Preintegration classes
        std::shared_ptr<PreintegrationType> preintegrated;
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> getPreintegrationParams();

        // Control
        bool is_init_ = false;

        double ts_head_;
        Vector3 prev_acc_;
        Vector3 prev_rate_;

        // Tunable parameters
        double gravity_norm_;

        double accel_noise_sigma_;
        double gyro_noise_sigma_;
        double accel_bias_rw_sigma_;
        double gyro_bias_rw_sigma_;

        double imu_attitude_sigma_;
};