#include "icetrack/navigation/GnssCorrection.h"

GnssCorrection::GnssCorrection(){}

GnssCorrection::GnssCorrection(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    gnss_ = system->gnss();

    // Get config
    double sigma = getParamOrThrow<double>(nh, "/navigation/gnss/sigma");
    correction_noise_ = noiseModel::Isotropic::Sigma(2, sigma);

    getParamOrThrow<double>(nh, "/navigation/gnss/gate_threshold", gate_threshold_);
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

void GnssCorrection::update(){
    const GnssMeasurement& meas = gnss_->getMeasurement();

    ts_head_ = meas.ts;
    xy_ = meas.xy;
}

bool GnssCorrection::gate(Point2 xy_est){
    return true;
    // if ((xy_ - xy_est).norm() > gate_threshold_){
    //     ROS_WARN("GnssCorrection: Measurement gate failed...");
    //     return false;
    // }
    // return true;
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