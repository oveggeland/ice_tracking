#include "icetrack/system/Gnss.h"

Gnss::Gnss(){
    // Default constructor
}

Gnss::Gnss(ros::NodeHandle nh){
    proj_ = Projection(nh);
    
    getParamOrThrow(nh, "/system/gnss/write_to_file", write_to_file_);

    if (write_to_file_){
        // Generate file path and make directories
        fs::path outpath = getParamOrThrow<std::string>(nh, "/outpath");
        fs::path fname = outpath / "navigation" / "sensors" / "gnss.csv";
        makePath(fname, true);

        f_out_ = std::ofstream(fname);
        f_out_ << "ts,latitude,longitude,altitude,x,y" << std::endl;
        f_out_ << std::fixed;
    }
}


bool Gnss::newMessage(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    gtsam::Vector2 xy = proj_.project(msg->latitude, msg->longitude);

    if (write_to_file_){
        f_out_ << ts;
        f_out_ << "," << msg->latitude << "," << msg->longitude << "," << msg->altitude;
        f_out_ << "," << xy.x() << "," << xy.y();
        f_out_ << std::endl;
    }

    meas_ = GnssMeasurement{
        ts,
        xy,
    };
    return true;
}

const GnssMeasurement& Gnss::getMeasurement() const{
    return meas_;
}