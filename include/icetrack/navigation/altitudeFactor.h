#ifndef ALTITUDEFACTOR_H
#define ALTITUDEFACTOR_H

#include <string.h>
#include <proj.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;

// Vanilla factor
class AltitudeFactor : public NoiseModelFactor1<Pose3> {
private:
    // Altitude measurement
    double altitude_measurement_;

public:
    // Constructor
    AltitudeFactor(Key poseKey, double altitude_measurement, const SharedNoiseModel& noiseModel)
        : NoiseModelFactor1<Pose3>(noiseModel, poseKey), altitude_measurement_(altitude_measurement) {}

    // Evaluate function
    virtual Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H) const override {
        // Calculate altitude from pose
        double altitude = pose.translation(H).z();

        // Compute error (residual)
        Vector1 error = Vector1(altitude - altitude_measurement_);

        if (H) {
            // Numerical differentiation to compute Jacobian
            //H->resize(1, 6);
            (*H) = H->block(2, 0, 1, 6);
        }

        return error;
    }
};




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


#endif