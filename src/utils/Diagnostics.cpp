#include "icetrack/Diagnostics.h"

Diagnostics::Diagnostics(){};

Diagnostics::Diagnostics(const std::string& f_out){
    f_out_ = std::ofstream(f_out);
    f_out_ << "t_stamp,t0_wall,t1_wall";
    f_out_ << std::endl << std::fixed;
};

void Diagnostics::diagStart(double ts){
    t1_wall_ = ros::Time::now().toSec();
    t_stamp_ = ts;
}

void Diagnostics::diagEnd(){
    t0_wall_ = ros::Time::now().toSec();

    writeDiag();
}

void Diagnostics::diagStep(double ts){
    diagEnd();
    diagStart(ts);
}

void Diagnostics::writeDiag(){
    f_out_ << t_stamp_ << "," << t0_wall_ << "," << t1_wall_ << std::endl;
}