#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Correction using ship navigation data
*/
class ShipPositionFactor : public NoiseModelFactor2<Pose3, Pose3> {
private:
    // No measurement
    Pose3 T_ship_;

public:
    // Constructor
    ShipPositionFactor(Key poseKey, Key alignmentKey, Pose3 T_ship, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Pose3>(noiseModel, poseKey, alignmentKey), T_ship_(T_ship) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& T_sys, const Pose3& T_align, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        if (H1 || H2){
            Point3 pos_pred = T_ship_.compose(T_align, boost::none, H2).translation();
            *H2 = -H2->bottomRows(3);

            auto error = T_sys.translation(H1) - pos_pred;
            return error;
        }
        else
            return T_sys.translation() - T_ship_.compose(T_align).translation();
    }
};