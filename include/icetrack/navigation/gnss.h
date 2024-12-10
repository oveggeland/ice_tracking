#pragma once

#include <proj.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <sensor_msgs/NavSatFix.h>

#include "icetrack/navigation/common.h"

class GnssHandle{
public:
  GnssHandle();

  void init(const sensor_msgs::NavSatFix::ConstPtr& msg);
  bool isInit();

  Point2 getPosition();
  Point2 getVelocity();

  boost::shared_ptr<gtsam::NonlinearFactor> getCorrectionFactor(const sensor_msgs::NavSatFix::ConstPtr& msg, Key key);

private:
  double ts_ = 0.0;
  Point2 xy_ = Point2::Zero();
  Point2 v_xy_ = Point2::Zero();

  bool init_ = false;

  Point2 getMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg);

  noiseModel::Isotropic::shared_ptr correction_noise_;
  PJ* projection_;
};


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
