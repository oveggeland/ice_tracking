#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

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

/* Defines a state in the pose graph */
struct PoseGraphState{
    size_t idx; // Unique state id (used for lookup)
    double ts;
    Pose3 pose;
    Vector3 velocity;
    imuBias::ConstantBias bias;
    Vector3 lever_arm;
};