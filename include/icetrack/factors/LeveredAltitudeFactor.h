#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Motion constraint between pose and lever arm.
*/
class LeveredAltitudeFactor : public NoiseModelFactor2<Pose3, Point3> {
private:
    // No measurement

public:
    // Constructor
    LeveredAltitudeFactor(Key poseKey, Key leverKey, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor2<Pose3, Point3>(noiseModel, poseKey, leverKey) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& pose, const Point3& bLbs, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const override {
        
        // Predicted altitude from lever arm and rotation
        Point3 wLws = pose.transformFrom(bLbs, H1, H2);

        if (H1 || H2) {
            *H1 = H1->bottomRows(1);
            *H2 = H2->bottomRows(1);
        }

        return Vector1(wLws[2]);
    }
};