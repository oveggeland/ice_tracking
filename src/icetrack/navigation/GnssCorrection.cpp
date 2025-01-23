#include "icetrack/navigation/GnssCorrection.h"

GnssCorrection::GnssCorrection(ros::NodeHandle nh, const Gnss& gnss) : gnss_(gnss){
    // Get config
    getParamOrThrow(nh, "/navigation/gnss/timeout_interval", timeout_interval_);
    getParamOrThrow(nh, "/navigation/gnss/suspension_interval", suspension_interval_);

    double sigma = getParamOrThrow<double>(nh, "/navigation/gnss/sigma_xy");
    correction_noise_ = noiseModel::Isotropic::Sigma(2, sigma);
}

bool GnssCorrection::initialize(){
    bool init = false;

    double ts = gnss_.getTimeStamp();
    Point2 xy = gnss_.getPosition();

    if (!xy_.isZero()){
        v_xy_ = (xy - xy_) / (ts - ts_);
        init = true;
    }

    ts_ = ts;
    xy_ = xy;

    return init;
}

bool GnssCorrection::update(){
    bool ret = false;
    double ts = gnss_.getTimeStamp();
    Point2 xy = gnss_.getPosition();

    if (ts - ts_ > timeout_interval_){  // Timeout, start suspension period
        ROS_WARN("GnssCorrection: Timeout detected");
        t0_suspend_ = ts;
    }
    else if (ts - t0_suspend_ > suspension_interval_){ // Suspension finished
        ret = true;
    }
    
    ts_ = ts;
    xy_ = xy;
    return ret;
}

GNSSFactor GnssCorrection::getCorrectionFactor(Key key) const{
    return GNSSFactor(key, xy_, correction_noise_);
}

double GnssCorrection::getHead() const {
    return ts_;
}

Point2 GnssCorrection::getVelocity() const{
    return v_xy_;
}

Point2 GnssCorrection::getPosition() const{
    return xy_;
}