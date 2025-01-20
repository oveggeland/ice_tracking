#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "icetrack/common.h"
#include "icetrack/navigation/navigation.h"
#include "icetrack/system/SensorSystem.h"


class ImuIntegration{
    public:
        // Constructors
        ImuIntegration();
        ImuIntegration(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system);

        // Interface
        void initialize();
        bool isInit() const;

        void integrate();
        void resetIntegration(double ts, imuBias::ConstantBias bias);
        NavState predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias) const;

        bool timeOut() const;
        double getHead() const;
        Rot3 estimateAttitude() const;
        
        CombinedImuFactor getIntegrationFactor(double ts_correction, int correction_count);
        Pose3AttitudeFactor getAttitudeFactor(Key key) const;

    private:
        // Imu class
        std::shared_ptr<const Imu> imu_;

        // Preintegration class
        std::shared_ptr<PreintegrationType> preintegration_;
        boost::shared_ptr<PreintegrationCombinedParams> getPreintegrationParams() const;

        // Control
        bool is_init_ = false;

        double ts_head_;
        double ts_tail_; 
        double timeout_interval_;

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