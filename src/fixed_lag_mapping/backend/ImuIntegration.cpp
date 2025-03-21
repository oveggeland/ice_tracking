#include "ImuIntegration.h"

ImuIntegration::ImuIntegration(ros::NodeHandle nh){
    // Get config
    getParamOrThrow(nh, "/imu/gravity_norm", gravity_norm_);
    getParamOrThrow(nh, "/imu/timeout_interval", timeout_interval_);

    getParamOrThrow(nh, "/imu/accel_noise_sigma", accel_noise_sigma_);
    getParamOrThrow(nh, "/imu/gyro_noise_sigma", gyro_noise_sigma_);
    getParamOrThrow(nh, "/imu/accel_bias_rw_sigma", accel_bias_rw_sigma_);
    getParamOrThrow(nh, "/imu/gyro_bias_rw_sigma", gyro_bias_rw_sigma_);

    getParamOrThrow(nh, "/imu/attitude_sigma", imu_attitude_sigma_);

    // Preintegration module (NB: Must be initialized after params above!!!)
    auto p = getPreintegrationParams();
    preintegration_ = std::make_shared<PreintegratedCombinedMeasurements>(p);

    assert(preintegration_);
}


void ImuIntegration::newMeasurement(const sensor_msgs::Imu::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();

    // Initialized?
    if (ts_head_ == 0.0)
        resetIntegration(ts, preintegration_->biasHat());

    // Timestamp
    double dt = ts - ts_head_;
    assert(dt >= 0);

    // All good, set new measurement
    setAcc(msg);
    setRate(msg);

    // Integrate and reset headers
    if (dt > 0)
        preintegration_->integrateMeasurement(acc_, rate_, dt);

    ts_head_ = ts;
}

void ImuIntegration::setAcc(const sensor_msgs::Imu::ConstPtr& msg){
    acc_ = Vector3(
        msg->linear_acceleration.x, 
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );
}

void ImuIntegration::setRate(const sensor_msgs::Imu::ConstPtr& msg){
    rate_ = Vector3(
        msg->angular_velocity.x, 
        msg->angular_velocity.y,
        msg->angular_velocity.z
    );
}

void ImuIntegration::resetIntegration(double ts, imuBias::ConstantBias bias){
    ts_head_ = ts;
    ts_tail_ = ts;
    preintegration_->resetIntegrationAndSetBias(bias);
}

void ImuIntegration::finishIntegration(double ts){
    double dt = ts - ts_head_;
    assert(dt >= 0);

    if (dt > 0) // Potentially finish integration if there is still a time delta :) 
        preintegration_->integrateMeasurement(acc_, rate_, dt);
}

/**
 * When correction occurs, we should finish integration by extrapolating the last imu measurements, and return a IMU factor
 */
CombinedImuFactor ImuIntegration::getIntegrationFactor(int state_idx){    
    auto preint_imu = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegration_);
    return CombinedImuFactor(
        X(state_idx-1), V(state_idx-1), 
        X(state_idx), V(state_idx), 
        B(state_idx-1), B(state_idx), 
        preint_imu);
}

Pose3AttitudeFactor ImuIntegration::getAttitudeFactor(Key key) const{
    return Pose3AttitudeFactor(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, imu_attitude_sigma_), Unit3(acc_));
}

void ImuIntegration::predictState(PoseGraphState& state) const{
    NavState pred = preintegration_->predict(NavState(state.pose, state.velocity), state.bias);
    state.pose = pred.pose();
    state.velocity = pred.velocity();
}

// Estimate attitude from last acceleration measurement
Rot3 ImuIntegration::estimateAttitude() const{
    Unit3 nA = Unit3(-acc_);
    Unit3 nG(0, 0, 1);

    return Rot3::AlignPair(nA.cross(nG), nG, nA);
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> ImuIntegration::getPreintegrationParams() const{
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