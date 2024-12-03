#include "icetrack/mapping/utils.h"

gtsam::Pose3 readExtrinsics(const char source, const char target, const std::string& filename){
    YAML::Node config = YAML::LoadFile(filename);

    gtsam::Matrix4 cTb_mat, lTc_mat;
    auto cTb_node = config["T_cam_imu"];
    auto lTc_node = config["T_lidar_cam"];
    
    
    // Fill the matrix with the values from the YAML node
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            cTb_mat(i, j) = cTb_node[i][j].as<double>();
            lTc_mat(i, j) = lTc_node[i][j].as<double>();
        }
    }
    gtsam::Pose3 cTb(cTb_mat);
    gtsam::Pose3 lTc(lTc_mat);

    // Return pose that transforms a point from source to target
    if (target == 'B' && source == 'C'){
        return cTb.inverse();
    }
    if (target == 'B' && source == 'L'){
        return cTb.inverse().compose(lTc.inverse());
    }

    if (target == 'C' && source == 'B'){
        return cTb;
    }
    if (target == 'C' && source == 'L'){
        return lTc.inverse();
    }

    if (target == 'L' == source == 'B'){
        return lTc.compose(cTb);
    }
    if (target == 'L' == source == 'C'){
        return lTc;
    }
    
    return gtsam::Pose3(); // Invalid request
}

// Function to query pose for a given time
bool queryPose(const std::map<double, gtsam::Pose3>& pose_map, double query_time, gtsam::Pose3& pose_out) {
    auto it = pose_map.lower_bound(query_time);
    
    if (it == pose_map.end() || it == pose_map.begin()) {
        return false; // Out of bounds, no pose available
    }
    
    double time1 = std::prev(it)->first;
    double time2 = it->first;

    double t = (query_time - time1) / (time2 - time1);
    
    const gtsam::Pose3& pose1 = std::prev(it)->second;
    const gtsam::Pose3& pose2 = it->second;
    
    pose_out = pose1.interpolateRt(pose2, t);
    return true;
}