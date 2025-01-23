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
        makePath(fname);

        f_out_ = std::ofstream(fname);
        f_out_ << "ts,latitude,longitude,altitude,x,y" << std::endl;
        f_out_ << std::fixed;
    }
}

/**
 * Parse incoming NavSatFix message and save the data.
 */
void Gnss::newMessage(const sensor_msgs::NavSatFix::ConstPtr& msg){
    ts_ = msg->header.stamp.toSec();
    xy_ = proj_.project(msg->latitude, msg->longitude);

    if (write_to_file_){
        f_out_ << ts_;
        f_out_ << "," << msg->latitude << "," << msg->longitude << "," << msg->altitude;
        f_out_ << "," << xy_.x() << "," << xy_.y();
        f_out_ << std::endl;
    }
}


const double& Gnss::getTimeStamp() const{
    return ts_;
};


const gtsam::Point2& Gnss::getPosition() const{
    return xy_;
};