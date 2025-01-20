#include "icetrack/gnss.h"

Gnss::Gnss(){}

Gnss::Gnss(ros::NodeHandle nh): nh_(nh){
    // Get config
    getParamOrThrow(nh_, "/nav/gnss_sigma_", gnss_sigma_);
    getParamOrThrow(nh_, "/nav/gnss_sigma_", gnss_sigma_);
    getParamOrThrow(nh_, "/nav/gnss_crs_source", crs_source_);
    getParamOrThrow(nh_, "/nav/gnss_crs_target", crs_target_);

    correction_noise_ = noiseModel::Isotropic::Sigma(2, gnss_sigma_);
    projection_ = proj_create_crs_to_crs(proj_context_create(), crs_source_.c_str(), crs_target_.c_str(), NULL);
}

boost::shared_ptr<gtsam::NonlinearFactor> Gnss::getCorrectionFactor(Point2 xy, Key key){
    return boost::make_shared<GNSSFactor>(key, xy, correction_noise_);
}


void Gnss::init(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    Point2 xy = getMeasurement(msg);

    if (!xy_.isZero()){
        v_xy_ = (xy - xy_) / (ts - ts_);
        init_ = true;
    }

    ts_ = ts;
    xy_ = xy;
}

bool Gnss::isInit(){
    return init_;
};

Point2 Gnss::getMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg){
    PJ_COORD input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    PJ_COORD output_coords = proj_trans(projection_, PJ_FWD, input_coords);
    
    return Point2(output_coords.xy.y, output_coords.xy.x);
}


Point2 Gnss::getVelocity(){
    return v_xy_;
}

Point2 Gnss::getPosition(){
    return xy_;
}