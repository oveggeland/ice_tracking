#include "icetrack/gnss.h"

GnssHandle::GnssHandle(){}

GnssHandle::GnssHandle(ros::NodeHandle nh): nh_(nh){
    // Get config
    getParamOrThrow(nh_, "/nav/gnss_sigma_", gnss_sigma_);
    getParamOrThrow(nh_, "/nav/gnss_sigma_", gnss_sigma_);
    getParamOrThrow(nh_, "/nav/gnss_crs_source", crs_source_);
    getParamOrThrow(nh_, "/nav/gnss_crs_target", crs_target_);

    correction_noise_ = noiseModel::Isotropic::Sigma(2, gnss_sigma_);
    projection_ = proj_create_crs_to_crs(proj_context_create(), crs_source_.c_str(), crs_target_.c_str(), NULL);
}

boost::shared_ptr<gtsam::NonlinearFactor> GnssHandle::getCorrectionFactor(const sensor_msgs::NavSatFix::ConstPtr& msg, Key key){
    return boost::make_shared<GNSSFactor>(key, getMeasurement(msg), correction_noise_);
}


void GnssHandle::init(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    Point2 xy = getMeasurement(msg);

    if (!xy_.isZero()){
        v_xy_ = (xy - xy_) / (ts - ts_);
        init_ = true;
    }

    ts_ = ts;
    xy_ = xy;
}

bool GnssHandle::isInit(){
    return init_;
};

Point2 GnssHandle::getMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg){
    PJ_COORD input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    PJ_COORD output_coords = proj_trans(projection_, PJ_FWD, input_coords);
    
    return Point2(output_coords.xy.y, output_coords.xy.x);
}


Point2 GnssHandle::getVelocity(){
    return v_xy_;
}

Point2 GnssHandle::getPosition(){
    return xy_;
}