#include "icetrack/mapping/processing.h"

CloudProcessor::CloudProcessor(){
    // Setup output file
    f_stats_ = std::ofstream("/home/oskar/icetrack/output/stats/stats.csv");
    f_stats_ << "ts,count,z_mean,z_var,z_min,z_max" << std::endl << std::fixed;
}


void CloudProcessor::processCloud(double ts, std::vector<point> points){
    ROS_INFO_STREAM(points.size() << ", " << points.capacity());
    cloud_ = std::move(points); // Assign the new points to the cloud object

    calculateMoments();

    f_stats_ << ts << "," << cloud_.size();
    f_stats_ << "," << z_mean_ << "," << z_var_ << "," << z_min_ << "," << z_max_;
    f_stats_ << std::endl;
}


void CloudProcessor::calculateMoments(){
    z_min_ = -__FLT_MAX__;
    z_max_ = __FLT_MAX__;
    
    double sum = 0;
    for (auto p: cloud_){
        sum += p.z;

        if (p.z < z_min_)
            z_min_ = p.z;
        else if (p.z > z_max_)
            z_max_ = p.z;
    }
    z_mean_ = sum / cloud_.size();

    sum = 0;
    for (auto p: cloud_){
        double err = p.z - z_mean_;
        sum += err*err;
    }
    z_var_ = sum / cloud_.size();
}