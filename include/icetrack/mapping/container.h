#pragma once

#include <ros/ros.h>
#include <stdint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <pcl/segmentation/sac_segmentation.h>

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
    PointCloudBuffer();
    PointCloudBuffer(size_t capacity);

    // Add a new element the ring buffer
    PointXYZIT* addPoint();
    void addPoint(const PointXYZIT& new_point);
    
    // Get the current points in the buffer (from oldest to newest)
    std::vector<PointXYZIT> getPoints() const;
    
    // Time interval query
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointsWithin(double t0, double t1);

    // Get the current size of the buffer (number of points stored)
    size_t size() const;

    // Get the capacity of the buffer (maximum number of points the buffer can hold)
    size_t capacity() const;

    // Check if the buffer is full
    bool isFull() const;

    // Check if the buffer is empty
    bool isEmpty() const;

    // Remove points with timestamp less than a threshold
    void removePointsBefore(double threshold);

    // Create a PCL cloud from the points in the buffer
    pcl::PointCloud<pcl::PointXYZI>::Ptr toPCLCloud() const;

private:
    size_t capacity_;              // Maximum number of points the buffer can hold
    size_t size_;                  // Current number of points in the buffer
    int head_;                  // Index of the next point to be added

    std::vector<PointXYZIT> buffer_;    // Buffer to store the points

    void increment(int& idx) const;
    void decrement(int& idx) const;

    int getTail() const;
};