#include "icetrack/navigation/gnss.h"

GnssHandle::GnssHandle(){
    // Noise
    correction_noise_ = noiseModel::Isotropic::Sigma(2, 2.0);
    projection_ = proj_create_crs_to_crs(proj_context_create(), "EPSG:4326", "EPSG:6052", NULL);
}

boost::shared_ptr<gtsam::NonlinearFactor> GnssHandle::getCorrectionFactor(Point2 xy, int correction_count){
    return boost::make_shared<GNSSFactor>(X(correction_count), xy, correction_noise_);
}

Point2 GnssHandle::getMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg){
    PJ_COORD input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    PJ_COORD output_coords = proj_trans(projection_, PJ_FWD, input_coords);
    
    return Point2(output_coords.xy.y, output_coords.xy.x);
}