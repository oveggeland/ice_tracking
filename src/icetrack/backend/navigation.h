#pragma once

#include <gtsam/inference/Symbol.h>

using namespace gtsam;

// State
using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

// Funky stuff
using symbol_shorthand::L;  // Point3 (Lever arm from body to ship buyonacy center, used for motion constraint)
using symbol_shorthand::F;  // Double (Freeboard estimate, used as "bias" for lidar based altitude measurements)

using symbol_shorthand::D; // Point2 - Ice drift  (north, east)

inline bool keyTypeCheck(Key key, char c){
    return Symbol(key).chr() == c;
}