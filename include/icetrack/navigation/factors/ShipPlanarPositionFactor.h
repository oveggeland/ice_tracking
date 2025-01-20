#pragma once

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Correction using ship navigation data
*/
class ShipPlanarPositionFactor : public NoiseModelFactor2<Pose3, Pose3> {
private:
    // No measurement
    Point2 xy_;

public:
    // Constructor
    ShipPlanarPositionFactor(Key poseKey, Key alignmentKey, Point2 xy, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Pose3>(noiseModel, poseKey, alignmentKey), xy_(xy) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& T_sys, const Pose3& T_align, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        if (H1 || H2){
            Matrix66 H_inv; 
            Point2 xy_pred = T_sys.compose(T_align.inverse(H_inv), H1, H2).translation().head<2>();

            *H1 = H1->block(3, 0, 2, 6);

            *H2 = *H2*H_inv;
            *H2 = H2->block(3, 0, 2, 6);

            return xy_pred - xy_;
        }
        else
            return T_sys.compose(T_align.inverse()).translation().head<2>() - xy_;
    }
};