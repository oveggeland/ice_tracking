#pragma once

#include <stdint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// This struct defines a LiDAR point
struct point {
    double x;
    double y;
    double z;
    uint8_t intensity;
    double ts;
};

// Ring buffer class
class RingBuffer {
public:
    // Constructors
    RingBuffer();
    RingBuffer(size_t capacity);

    // Add a new point to the ring buffer
    void addPoint(const point& new_point);

    // Get the current points in the buffer (from oldest to newest)
    std::vector<point> getPoints() const;

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
    std::vector<point> buffer_;    // Buffer to store the points
    size_t head_;                  // Index of the next point to be added
    size_t size_;                  // Current number of points in the buffer
};
