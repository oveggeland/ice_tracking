#pragma once

#include "ros/ros.h"
#include "gtsam/geometry/Pose3.h"

#include "icetrack/utils/utils.h"
#include "icetrack/ShipNavigation.h"
#include "icetrack/navigation/Projection.h"

class Ship{
    public:
        Ship();
        Ship(ros::NodeHandle nh);

        bool newMessage(const icetrack::ShipNavigation::ConstPtr& msg);
        gtsam::Pose3 predictSystemPose() const;

        double getTimeStamp() const;
        const gtsam::Pose3& getShipPose() const;
    
    private:
        Projection proj_;

        // Alignment transformation (T_sys = T_ship * T_align)
        gtsam::Pose3 T_align_;
        
        // State
        double ts_;
        gtsam::Pose3 T_ship_;
};