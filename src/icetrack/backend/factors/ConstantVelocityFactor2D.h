#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

/*
Motion constraint between pose and lever arm.
*/
class ConstantVelocityFactor2D : public NoiseModelFactor4<Point2, Point2, Point2, Point2> {
private:
    // No measurement
    double dt_;

public:
    // Constructor
    ConstantVelocityFactor2D (Key pos0Key, Key pos1Key, Key vel0Key, Key vel1Key, double dt)
        : NoiseModelFactor4<Point2, Point2, Point2, Point2>(noiseModel::Diagonal::Sigmas(
            Vector4(1.0e-9, 1.0e-9, 0.05*dt, 0.05*dt)), 
        pos0Key, pos1Key, vel0Key, vel1Key), dt_(dt){}

    virtual Vector evaluateError(const Point2& p0, const Point2& p1, const Point2& v0, const Point2& v1,
                                 boost::optional<Matrix&> H1, boost::optional<Matrix&> H2,
                                 boost::optional<Matrix&> H3, boost::optional<Matrix&> H4) const override {
        // Predicted position at time 1 (based on constant velocity model)
        Point2 p1_pred = p0 + v0 * dt_;
        Point2 v1_pred = v0;  // Assuming velocity is constant, no change in velocity

        // Compute error (difference between predicted and actual)
        Vector4 error;
        error << p1_pred - p1, v1_pred - v1;

        // Jacobians with respect to p0 (position at time 0)
        if (H1) {
            H1->resize(4, 2);
            H1->topLeftCorner<2, 2>() = Matrix2::Identity();
            H1->bottomLeftCorner<2, 2>() =  Matrix2::Zero();
        }
        // Jacobians with respect to p1 (position at time 1)
        if (H2) {
            H2->resize(4, 2);
            H2->topLeftCorner<2, 2>() = -Matrix2::Identity();
            H2->bottomLeftCorner<2, 2>() =  Matrix2::Zero();
        }
        // Jacobians with respect to v0 (velocity at time 0)
        if (H3) {
            H3->resize(4, 2);
            H3->topLeftCorner<2, 2>() = Matrix2::Identity()*dt_;
            H3->bottomLeftCorner<2, 2>() = Matrix2::Identity();
        }
        // Jacobians with respect to v1 (velocity at time 1)
        if (H4) {
            H4->resize(4, 2);
            H4->topLeftCorner<2, 2>() = Matrix2::Zero();           // Top half as Zero
            H4->bottomLeftCorner<2, 2>() = -Matrix2::Identity();   // Bottom half as Negative Identity
        }
        return error;
    }
};