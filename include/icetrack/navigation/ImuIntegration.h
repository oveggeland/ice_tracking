#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "icetrack/utils/utils.h"
#include "icetrack/navigation/navigation.h"
#include "icetrack/system/SensorSystem.h"


class ImuIntegration{
    public:
        // Constructors
        ImuIntegration(ros::NodeHandle nh, const Imu& imu_);

        // Interface
        void initialize();
        void integrate();
        void resetIntegration(double ts, imuBias::ConstantBias bias);
        NavState predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias) const;

        bool isInit() const { return init_; }
        bool timeOut() const { return (ts_head_ - ts_tail_) > timeout_interval_; };
        double getHead() const { return ts_head_; }
        Rot3 estimateAttitude() const;
        
        CombinedImuFactor getIntegrationFactor(double ts_correction, int correction_count);
        Pose3AttitudeFactor getAttitudeFactor(Key key) const;

    private:
        // Imu class
        const Imu& imu_;

        // Preintegration class
        std::shared_ptr<PreintegrationType> preintegration_;
        boost::shared_ptr<PreintegrationCombinedParams> getPreintegrationParams() const;

        // Control
        bool init_ = false;

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