#include "icetrack/navigation/gnss.h"

GnssHandle::GnssHandle(){
    // Noise
    correction_noise_ = noiseModel::Isotropic::Sigma(2, 1.0);
    projection_ = proj_create_crs_to_crs(proj_context_create(), "EPSG:4326", "EPSG:6052", NULL);
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


Point3 GnssHandle::getPriorVelocity(){
    return Point3(v_xy_.x(), v_xy_.y(), 0);
}

Point3 GnssHandle::getPriorPosition(){
    return Point3(xy_.x(), xy_.y(), 0);
}