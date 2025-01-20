#include "icetrack/system/Gnss.h"

Gnss::Gnss(){
    // Default constructor
}

Gnss::Gnss(ros::NodeHandle nh){
    proj_ = Projection(nh);

    getParamOrThrow(nh, "/system/gnss/vel_norm_threshold", vel_norm_threshold_);
    getParamOrThrow(nh, "/system/gnss/altitude_rate_threshold", altitude_rate_threshold_);
}


bool Gnss::newMessage(const sensor_msgs::NavSatFix::ConstPtr& msg){
    // Sanity check 1: Time
    double ts = msg->header.stamp.toSec();
    if (ts <= meas_.ts){
        ROS_WARN("Gnss: Timestamp is prior or equal to previous GNSS message...");
        return false;
    }

    // Sanity check 2: Implied velocity
    gtsam::Vector2 xy = proj_.project(msg->latitude, msg->longitude);
    gtsam::Vector2 v_est = (xy - meas_.xy) / (ts - meas_.ts);

    if (meas_.ts > 0.0 && v_est.norm() > vel_norm_threshold_){
        ROS_WARN("Gnss: Measurements implies a velocity which is higher than the accepted threshold...");
        return false;
    }

    // Sanity check 3: Is altitude not changing too much?
    double altitude = msg->altitude;
    double dz_dt = (altitude - meas_.altitude) / (ts - meas_.ts);

    if (meas_.ts > 0.0 && abs(dz_dt) > altitude_rate_threshold_){
        ROS_WARN("Gnss: Altitude change is higher than the accepted threshold...");
        return false;
    }

    ROS_INFO_STREAM(std::fixed << gtsam::Vector3(xy.x(), xy.y(), altitude).transpose());

    meas_ = GnssMeasurement{
        ts,
        xy,
        altitude
    };
    return true;
}

const GnssMeasurement& Gnss::getMeasurement() const{
    return meas_;
}