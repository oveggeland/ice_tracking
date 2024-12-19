#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;


class GNSSFactor: public NoiseModelFactor1<Pose3> {
  private:
    Point2 measured_; /** The measurement */
  public:
    /** Constructor */
    GNSSFactor(Key posekey, const Point2 measured,
        const SharedNoiseModel& model) :
      NoiseModelFactor1<Pose3>(model, posekey), measured_(measured) {
    }

    /** vector of errors */
    Vector evaluateError(const Pose3& pose,
        boost::optional<Matrix&> H) const override {

      if (H){
        H->resize(2,6); // jacobian wrt pose
        (*H) << Z_3x3.block(0, 0, 2, 3),  pose.rotation().matrix().block(0, 0, 2, 3);
      }
      return pose.translation().head<2>() - measured_;
    }
  }; // \class GNSSFactor
