#pragma once

#include <ros/ros.h>
#include <stdint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


struct PointXYZIT {
    double x;
    double y;
    double z;
    float intensity;
    double ts;
};

// Buffer to store point cloud
class PointCloudBuffer {
public:
    // Constructors
    PointCloudBuffer(){};
    PointCloudBuffer(size_t capacity);

    // Add a new element the ring buffer
    PointXYZIT* addPoint();
    void addPoint(const PointXYZIT& new_point);
    
    // Time interval query
    std::vector<PointXYZIT> getPointsWithin(double t0, double t1) const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPclWithin(double t0, double t1) const;

    // Remove points with timestamp less than a threshold
    void removePointsBefore(double threshold);

    // Getters
    size_t size() const;
    size_t capacity() const;
    bool isFull() const;
    bool isEmpty() const;

private:
    size_t capacity_;              // Maximum number of points the buffer can hold
    size_t size_;                  // Current number of points in the buffer
    int head_;                  // Index of the next point to be added

    std::vector<PointXYZIT> buffer_;    // Buffer to store the points

    void increment(int& idx) const;
    void decrement(int& idx) const;

    int idxAdd(int idx, int offset) const;
    int idxDiff(int idx0, int idx1) const;
    int idxLowerBound(double ts) const;

    int getTail() const;
};