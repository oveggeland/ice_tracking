#include "icetrack/navigation/GnssCorrection.h"

GnssCorrection::GnssCorrection(){}

GnssCorrection::GnssCorrection(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    gnss_ = system->gnss();

    // Get config
    getParamOrThrow(nh, "/navigation/gnss/timeout_interval", timeout_interval_);
    getParamOrThrow(nh, "/navigation/gnss/suspension_interval", suspension_interval_);

    double sigma = getParamOrThrow<double>(nh, "/navigation/gnss/sigma_xy");
    correction_noise_ = noiseModel::Isotropic::Sigma(2, sigma);
}

bool GnssCorrection::initialize(){
    bool ret = false;
    const GnssMeasurement& meas = gnss_->getMeasurement();

    if (!xy_.isZero()){
        v_xy_ = (meas.xy - xy_) / (meas.ts - ts_head_);
        ret = true;
    }

    ts_head_ = meas.ts;
    xy_ = meas.xy;

    return ret;
}

bool GnssCorrection::update(){
    const GnssMeasurement& meas = gnss_->getMeasurement();

    bool ret = false;
    if (meas.ts - ts_head_ > timeout_interval_){  // Timeout, start suspension period
        ROS_WARN("GnssCorrection: Timeout detected");
        t0_suspend_ = meas.ts;
    }
    else if (meas.ts - t0_suspend_ > suspension_interval_){ // Suspension finished
        ret = true;
    }
    else{
        ROS_WARN("GnssCorrection: Waiting for suspension to finish");
    }
    
    ts_head_ = meas.ts;
    xy_ = meas.xy;
    return ret;
}

GNSSFactor GnssCorrection::getCorrectionFactor(Key key) const{
    return GNSSFactor(key, xy_, correction_noise_);
}

double GnssCorrection::getHead() const {
    return ts_head_;
}

Point2 GnssCorrection::getVelocity() const{
    return v_xy_;
}

Point2 GnssCorrection::getPosition() const{
    return xy_;
}