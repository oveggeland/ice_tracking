#include "GnssCorrection.h"

GnssCorrection::GnssCorrection(ros::NodeHandle nh){
    getParamOrThrow(nh, "/navigation/gnss/timeout_interval", timeout_interval_);
    getParamOrThrow(nh, "/navigation/gnss/suspension_interval", suspension_interval_);

    double sigma = getParamOrThrow<double>(nh, "/navigation/gnss/sigma_xy");
    correction_noise_ = noiseModel::Isotropic::Sigma(2, sigma);

    // Projection stuff
    std::string crs_source = getParamOrThrow<std::string>(nh, "/navigation/crs_source");
    std::string crs_target = getParamOrThrow<std::string>(nh, "/navigation/crs_target");

    projection_ = proj_create_crs_to_crs(proj_context_create(), crs_source.c_str(), crs_target.c_str(), NULL);
}


void GnssCorrection::newMeasurement(const sensor_msgs::NavSatFix::ConstPtr msg){
    double ts = msg->header.stamp.toSec();
    Point2 xy = project(msg);

    // Check validity of measurement
    fix_ = checkFix(ts, xy);
    ts_ = ts;
    xy_ = xy;
}


Point2 GnssCorrection::project(const sensor_msgs::NavSatFix::ConstPtr msg) const{
    PJ_COORD input_coords = proj_coord(msg->latitude, msg->longitude, 0, 0);
    PJ_COORD output_coords = proj_trans(projection_, PJ_FWD, input_coords);
    
    return Point2(output_coords.xy.y, output_coords.xy.x);
}


bool GnssCorrection::checkFix(double ts, Point2 xy){
    if (ts_ != 0.0 && ts - ts_ > timeout_interval_){  // Timeout, start suspension period
        ROS_WARN("GnssCorrection: Timeout detected");
        t0_suspend_ = ts;
        return false;
    }
    else if (ts - t0_suspend_ < suspension_interval_){ // Still suspended
        return false;
    }

    return true;
}