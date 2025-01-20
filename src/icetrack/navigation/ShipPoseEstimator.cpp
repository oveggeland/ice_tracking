#include "icetrack/navigation/ShipPoseEstimator.h"

ShipPoseEstimator::ShipPoseEstimator(){}


ShipPoseEstimator::ShipPoseEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    proj_ = Projection(nh);
    ship_ = system->ship();
}


bool ShipPoseEstimator::shipUpdate(){
    ROS_WARN("ShipPoseEsimator::shipUpdate() not implemented");
    return false;
}


Pose3 ShipPoseEstimator::getShipPose(const icetrack::ShipNavigation::ConstPtr& msg) const{
    // Translation
    Point2 xy = proj_.project(msg->lat, msg->lng);
    Point3 t_ship = Point3 (xy.x(), xy.y(), msg->heave); // TODO: Is heave positive or negative?

    // Rotation
    Rot3 R_ship = Rot3::RzRyRx(DEG2RAD(msg->roll), DEG2RAD(msg->pitch), DEG2RAD(msg->heading));
    
    return Pose3(R_ship, t_ship);
}


Pose3 ShipPoseEstimator::predictSystemPose(const icetrack::ShipNavigation::ConstPtr& msg) const{
    Pose3 T_ship = getShipPose(msg);
    return T_ship.compose(T_align_);
}