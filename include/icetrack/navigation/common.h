#pragma once

#include <gtsam/inference/Symbol.h>

using namespace gtsam;

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::L;  // Point3 (bLbs - lever arm from body to ship buyonacy center)

#define DEG2RAD(degrees) ((degrees) * M_PI / 180.0)
#define RAD2DEG(radians) ((radians) * 180.0 / M_PI)