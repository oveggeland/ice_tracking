#pragma once

#include <gtsam/geometry/Pose3.h>
#include <yaml-cpp/yaml.h>

gtsam::Pose3 readExtrinsics(const char source, const char target, const std::string& filename);

bool queryPose(const std::map<double, gtsam::Pose3>& pose_map, double query_time, gtsam::Pose3& pose_out);