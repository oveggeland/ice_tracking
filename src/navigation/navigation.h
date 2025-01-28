#pragma once

#include <gtsam/inference/Symbol.h>

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::L;  // Point3 (bLbs - lever arm from body to ship buyonacy center)