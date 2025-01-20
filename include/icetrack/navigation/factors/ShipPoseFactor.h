#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Correction using ship navigation data
*/
class ShipPoseFactor : public NoiseModelFactor2<Pose3, Pose3> {
private:
    // No measurement
    Pose3 T_ship_;

public:
    // Constructor
    ShipPoseFactor(Key poseKey, Key alignmentKey, Pose3 T_ship, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Pose3>(noiseModel, poseKey, alignmentKey), T_ship_(T_ship) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& T_sys, const Pose3& T_align, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        if (H1 || H2){
            Matrix66 H2a, H2b;
            Pose3 T_pred = T_ship_.compose(T_align, boost::none, H2a);

            auto error = Pose3::Logmap(T_sys.between(T_pred, H1, H2b));

            *H2 = H2b*H2a;

            *H1 = H1->bottomRows(3);
            *H2 = H2->bottomRows(3);

            return error.tail<3>();
        }
        else
            return Pose3::Logmap(T_sys.between(T_ship_.compose(T_align))).tail<3>();
    }
};