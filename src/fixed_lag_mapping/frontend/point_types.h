#pragma once

// Used in point buffer
struct PointXYZIT{
    float x;
    float y;
    float z;
    float i;
    double ts;
};
static_assert(sizeof(PointXYZIT) == 24, "PointXYZIT struct has unexpected padding!");

// Packed point used in map (and compatible with ROS PointCloud2 publishing)
struct PointXYZI{
    float x;
    float y;
    float z;
    float i;
};
static_assert(sizeof(PointXYZI) == 16, "PointXYZI struct has unexpected padding!");