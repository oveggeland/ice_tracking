#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;


class NormConstraintFactor : public gtsam::NoiseModelFactor1<gtsam::Point3> {
private:
  double maxNorm_;

public:
  NormConstraintFactor(gtsam::Key key, double maxNorm, const gtsam::SharedNoiseModel& noiseModel)
    : NoiseModelFactor1<gtsam::Point3>(noiseModel, key), maxNorm_(maxNorm) {}

  gtsam::Vector evaluateError(const gtsam::Point3& point,
                              boost::optional<gtsam::Matrix&> H = boost::none) const override {
    double currentNorm = point.norm();
    
    if (currentNorm <= maxNorm_) {
      if (H) *H = gtsam::Matrix::Zero(1, 3);
      return gtsam::Vector1::Zero();
    }
    
    gtsam::Vector1 error;
    error << currentNorm - maxNorm_;
    
    if (H) {
      *H = point.transpose() / currentNorm;
    }
    
    return error;
  }
};