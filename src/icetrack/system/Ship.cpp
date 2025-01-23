#include "icetrack/system/Ship.h"

Ship::Ship(){
    // Default constructor
}

Ship::Ship(ros::NodeHandle nh){
    proj_ = Projection(nh);
    
    // Initialize ship to body alignment matrix
    std::vector<double> alignment_rotation = getParamOrThrow<std::vector<double>>(nh, "/system/ship/alignment_rotation");
    std::vector<double> alignment_translation = getParamOrThrow<std::vector<double>>(nh, "/system/ship/alignment_translation");
    T_align_ = gtsam::Pose3(
        gtsam::Rot3::RzRyRx(DEG2RAD(alignment_rotation[0]), DEG2RAD(alignment_rotation[1]), DEG2RAD(alignment_rotation[2])),
        gtsam::Point3(alignment_translation[0], alignment_translation[1], alignment_translation[2])
    );
}

double Ship::getTimeStamp() const{
    return ts_;
};

const gtsam::Pose3& Ship::getShipPose() const{
    return T_ship_;
};

gtsam::Pose3 Ship::predictSystemPose() const{
    return T_ship_.compose(T_align_);
};

bool Ship::newMessage(const icetrack::ShipNavigation::ConstPtr& msg){
        // Translation
    gtsam::Point2 xy = proj_.project(msg->lat, msg->lng);
    gtsam::Point3 t_ship = gtsam::Point3 (xy.x(), xy.y(), msg->heave); // TODO: Is heave positive or negative?

    // Rotation
    gtsam::Rot3 R_ship = gtsam::Rot3::RzRyRx(DEG2RAD(msg->roll), DEG2RAD(msg->pitch), DEG2RAD(msg->heading));
    T_ship_ = gtsam::Pose3(R_ship, t_ship);

    return true;
}