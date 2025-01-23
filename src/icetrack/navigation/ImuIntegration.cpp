#include "icetrack/navigation/ImuIntegration.h"

ImuIntegration::ImuIntegration(ros::NodeHandle nh, const Imu& imu) : imu_(imu) {
    // Get config
    getParamOrThrow(nh, "/navigation/gravity_norm", gravity_norm_);
    getParamOrThrow(nh, "/navigation/imu_timeout_interval", timeout_interval_);

    getParamOrThrow(nh, "/navigation/accel_noise_sigma", accel_noise_sigma_);
    getParamOrThrow(nh, "/navigation/gyro_noise_sigma", gyro_noise_sigma_);
    getParamOrThrow(nh, "/navigation/accel_bias_rw_sigma", accel_bias_rw_sigma_);
    getParamOrThrow(nh, "/navigation/gyro_bias_rw_sigma", gyro_bias_rw_sigma_);

    getParamOrThrow(nh, "/navigation/imu_attitude_sigma", imu_attitude_sigma_);

    // Preintegration module (NB: Must be initialized after params above!!!)
    auto p = getPreintegrationParams();
    preintegration_ = std::make_shared<PreintegratedCombinedMeasurements>(p);

    assert(preintegration_);
}


void ImuIntegration::initialize(){
    prev_acc_ = imu_.getAcc();
    prev_rate_ = imu_.getRate();
    init_ = true;
}

void ImuIntegration::integrate(){
    double ts = imu_.getTimeStamp();

    double dt = ts - ts_head_;
    assert(dt >= 0 && dt < 0.02);

    if (dt > 0){
        ts_head_ = ts;
        prev_acc_ = imu_.getAcc();
        prev_rate_ = imu_.getRate();
    
        preintegration_->integrateMeasurement(prev_acc_, prev_rate_, dt);
    }
}

void ImuIntegration::resetIntegration(double ts, imuBias::ConstantBias bias){
    ts_head_ = ts;
    ts_tail_ = ts;
    preintegration_->resetIntegrationAndSetBias(bias);
}

/**
 * When correction occurs, we should finish integration by extrapolating the last imu measurements, and return a IMU factor
 */
CombinedImuFactor ImuIntegration::getIntegrationFactor(double ts_correction, int correction_count){
    double dt = ts_correction - ts_head_;
    assert(dt >= 0 && dt < 0.02);

    if (dt > 0)
        preintegration_->integrateMeasurement(prev_acc_, prev_rate_, dt);
    
    auto preint_imu = dynamic_cast<const PreintegratedCombinedMeasurements&>(*preintegration_);
    return CombinedImuFactor(
        X(correction_count-1), V(correction_count-1), 
        X(correction_count), V(correction_count), 
        B(correction_count-1), B(correction_count), 
        preint_imu);
}

NavState ImuIntegration::predict(Pose3 pose, Point3 vel, imuBias::ConstantBias bias) const{
    return preintegration_->predict(NavState(pose, vel), bias);
}

// Estimate attitude from last acceleration measurement
Rot3 ImuIntegration::estimateAttitude() const{
    Unit3 nA = Unit3(-prev_acc_);
    Unit3 nG(0, 0, 1);

    return Rot3::AlignPair(nA.cross(nG), nG, nA);
}

Pose3AttitudeFactor ImuIntegration::getAttitudeFactor(Key key) const{
    return Pose3AttitudeFactor(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, imu_attitude_sigma_), Unit3(-prev_acc_));
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