#include "icetrack/imu.h"

// Extract accelerometer vector from Imu message
Vector3 getAcc(const sensor_msgs::Imu::ConstPtr& msg){
    return Vector3(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
}

// Extract gyroscope angular velocity vector from Imu message
Vector3 getRate(const sensor_msgs::Imu::ConstPtr& msg){
    return Vector3(
        msg->angular_velocity.x, 
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
}

Imu::Imu(){}

Imu::Imu(ros::NodeHandle nh): nh_(nh){
    // Get config
    getParamOrThrow(nh_, "/nav/gravity_norm", gravity_norm_);
    getParamOrThrow(nh_, "/nav/accel_noise_sigma", accel_noise_sigma_);
    getParamOrThrow(nh_, "/nav/gyro_noise_sigma", gyro_noise_sigma_);
    getParamOrThrow(nh_, "/nav/accel_bias_rw_sigma", accel_bias_rw_sigma_);
    getParamOrThrow(nh_, "/nav/gyro_bias_rw_sigma", gyro_bias_rw_sigma_);
    getParamOrThrow(nh_, "/nav/imu_attitude_sigma", imu_attitude_sigma_);

    getParamOrThrow(nh_, "/nav/imu_timeout_interval", timeout_interval_);

    auto p = getPreintegrationParams();

    preintegrated = std::make_shared<PreintegratedCombinedMeasurements>(p);
    assert(preintegrated);
}

void Imu::resetIntegration(double ts, imuBias::ConstantBias bias){
    ts_head_ = ts;
    ts_tail_ = ts;
    preintegrated->resetIntegrationAndSetBias(bias);
}

/**
 * Used after initialization of the navigation system. Every new IMU measurement should be integrated in waiting for a new correction.
 */
void Imu::integrate(const sensor_msgs::Imu::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    double dt = ts - ts_head_;

    assert(dt >= 0 && dt < 0.02);

    if (dt > 0){
        preintegrated->integrateMeasurement(prev_acc_, prev_rate_, dt);
        prev_acc_ = getAcc(msg);
        prev_rate_ = getRate(msg);
        ts_head_ = ts;
    }
}


/**
 * Check if IMU integration interval is exceeding the predefined timeout interval.
 * This typically happens when GNSS measurements are not available, at which point 
 * we should still add new state variables at a reasonable interval.
 */
bool Imu::timeOut(){
    return (ts_head_ - ts_tail_) > timeout_interval_;
}

/**
 * When correction occurs, we should finish integration by extrapolating the last imu measurements, and return a IMU factor
 */
CombinedImuFactor Imu::finishIntegration(double ts_correction, int correction_count){
    double dt = ts_correction - ts_head_;
    assert(dt >= 0 && dt < 0.02);

    if (dt > 0)
        preintegrated->integrateMeasurement(prev_acc_, prev_rate_, dt);
    
    auto preint_imu = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegrated);
    return CombinedImuFactor(
        X(correction_count-1), V(correction_count-1), 
        X(correction_count), V(correction_count), 
        B(correction_count-1), B(correction_count), 
        preint_imu);
}

NavState Imu::predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias){
    return preintegrated->predict(NavState(pose, vel), bias);
}


void Imu::init(const sensor_msgs::Imu::ConstPtr& msg){
    prev_acc_ = getAcc(msg);
    prev_rate_ = getRate(msg);
    is_init_ = true;
}

Unit3 Imu::getNz(){
    return Unit3(-prev_acc_);
}


boost::shared_ptr<gtsam::NonlinearFactor> Imu::getAttitudeFactor(Key key){
    return boost::make_shared<Pose3AttitudeFactor>(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, imu_attitude_sigma_), getNz());
}


// Estimate attitude from last acceleration measurement
Rot3 Imu::getPriorRot(){
    Unit3 nA = Unit3(-prev_acc_);
    Unit3 nG(0, 0, 1);

    return Rot3::AlignPair(nA.cross(nG), nG, nA);
}


boost::shared_ptr<gtsam::PreintegrationCombinedParams> Imu::getPreintegrationParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma_, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma_, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma_, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma_, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(gravity_norm_);
  
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}