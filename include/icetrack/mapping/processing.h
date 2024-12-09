#pragma once

#include <ros/ros.h>

#include <fstream>
#include <random>
#include "icetrack/mapping/container.h"



class CloudProcessor{
public:
    CloudProcessor();

    void processCloud(double ts, std::vector<point> points);
private:
    std::ofstream f_stats_;

    std::vector<point> cloud_;

    int count_;

    double z_mean_, z_var_, z_min_, z_max_;
    void calculateMoments();

    void meanElevationFilter();
    void ransacElevationFilter();

    // Fnc 2
    
    // Fnc 3
};