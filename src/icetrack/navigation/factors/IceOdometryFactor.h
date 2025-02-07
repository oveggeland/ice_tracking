#pragma once

#include <ros/ros.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Ice odometry factor, compensating for ice drift. We model the ice reference frame as a fixed translational offset from the world frame (assuming only planar motion in xy plane)
*/
class IceOdometryFactor : public NoiseModelFactor3<Pose3, Pose3, Point2> {
private:
    double dt_;
    Pose3 T_odom_;

public:
    // Constructor
    IceOdometryFactor(Key pose0_key, Key pose1_key, Key drift_key, Pose3 T_odom, double dt, const SharedNoiseModel &model)
        : NoiseModelFactor3<Pose3, Pose3, Point2>(model, pose0_key, pose1_key, drift_key), T_odom_(T_odom), dt_(dt) {}

    Vector evaluateError(const Pose3& i0Tb0, const Pose3& i1Tb1, const Point2& ice_drift,
                         boost::optional<Matrix&> H1, boost::optional<Matrix&> H2, boost::optional<Matrix&> H3) const override {
        Pose3 T_drift(Rot3(), dt_ * Point3(ice_drift.x(), ice_drift.y(), 0));
        Pose3 i0Tb1 = T_drift.compose(i1Tb1);
        Pose3 odom_pred = i0Tb0.between(i0Tb1);
        Pose3 T_dev = odom_pred.between(T_odom_);
        Vector6 error = Pose3::Logmap(T_dev);

        if (H1 || H2 || H3) {
            auto errorFunc = [&](const Pose3& i0Tb0, const Pose3& i1Tb1, const Point2& ice_drift) {
                return this->evaluateError(i0Tb0, i1Tb1, ice_drift, boost::none, boost::none, boost::none);
            };

            if (H1) *H1 = numericalDerivative31<Vector6, Pose3, Pose3, Point2>(errorFunc, i0Tb0, i1Tb1, ice_drift, 1e-5);
            if (H2) *H2 = numericalDerivative32<Vector6, Pose3, Pose3, Point2>(errorFunc, i0Tb0, i1Tb1, ice_drift, 1e-5);
            if (H3) *H3 = numericalDerivative33<Vector6, Pose3, Pose3, Point2>(errorFunc, i0Tb0, i1Tb1, ice_drift, 1e-5);
        }

        return error;
    }
};
