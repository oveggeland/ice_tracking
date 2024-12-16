#include "ros/ros.h"
#include <fstream>

// Class for diagnostics and performance
class Diagnostics{
public:
    Diagnostics();
    Diagnostics(const std::string& f_out);

    void diagStep(double ts);
    void diagStart(double ts);
    void diagEnd();

private:
    std::ofstream f_out_;
    void writeDiag();

    double t0_wall_, t1_wall_; // Wall time at diagStart and diagEnd
    double t_stamp_; // Message time stamp
    int msg_type_;   // Not used
    double ram_usage_; // Not used
};