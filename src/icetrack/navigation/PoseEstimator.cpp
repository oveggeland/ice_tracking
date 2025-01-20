#include "icetrack/navigation/PoseEstimator.h"

// Constructors
PoseEstimator::PoseEstimator(){}

PoseEstimator::PoseEstimator(ros::NodeHandle nh, std::shared_ptr<SensorSystem> system){
    inertial_pose_estimator_ = InertialPoseEstimator(nh, system);
    ship_pose_estimator_ = ShipPoseEstimator(nh, system);

    getParamOrThrow(nh, "/navigation/use_ship_estimate", use_ship_estimate_);
}

bool PoseEstimator::imuUpdate(){
    return inertial_pose_estimator_.imuUpdate();
}

bool PoseEstimator::gnssUpdate(){
    return inertial_pose_estimator_.gnssUpdate();
}

bool PoseEstimator::lidarUpdate(){
    return inertial_pose_estimator_.lidarUpdate();
}

bool PoseEstimator::shipUpdate(){
    return false;
    // return ship_pose_estimator_.shipUpdate();
}

Pose3 PoseEstimator::getPose() const{
    // if (use_ship_estimate_)
    //     return ship_pose_estimator_.getPose();
    return inertial_pose_estimator_.getPose();
}