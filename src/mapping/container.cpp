#include "icetrack/mapping/container.h"

RingBuffer::RingBuffer() : capacity_(0), head_(0), size_(0) {}

RingBuffer::RingBuffer(size_t capacity)
    : capacity_(capacity), buffer_(capacity), head_(0), size_(0) {}

void RingBuffer::addPoint(const point& new_point) {
    buffer_[head_] = new_point;
    head_ = (head_ + 1) % capacity_;

    if (size_ < capacity_) {
        ++size_;
    }
}

std::vector<point> RingBuffer::getPoints() const {
    std::vector<point> points(size_);

    size_t idx = (head_ - size_ + capacity_) % capacity_;
    for (size_t i = 0; i < size_; ++i) {
        points[i] = buffer_[(idx + i) % capacity_];  // Direct assignment
    }

    return points;
}

size_t RingBuffer::size() const {
    return size_;
}

size_t RingBuffer::capacity() const {
    return capacity_;
}

bool RingBuffer::isFull() const {
    return size_ == capacity_;
}

bool RingBuffer::isEmpty() const {
    return size_ == 0;
}

void RingBuffer::removePointsBefore(double threshold) {
    if (isEmpty()) {
        return;
    }

    size_t tailIndex = (head_ - size_ + capacity_) % capacity_;

    size_t newTailIndex = tailIndex;
    while (size_ > 0 && buffer_[newTailIndex].ts < threshold) {
        newTailIndex = (newTailIndex + 1) % capacity_;
        --size_;
    }
}


pcl::PointCloud<pcl::PointXYZI>::Ptr RingBuffer::toPCLCloud() const {
    // Create a new PCL cloud
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Reserve space for the points
    cloud->points.resize(size_);

    // Directly write into the points vector
    size_t tailIndex = (head_ - size_ + capacity_) % capacity_;
    for (size_t i = 0; i < size_; ++i) {
        const point& pt = buffer_[(tailIndex + i) % capacity_];
        cloud->points[i].x = pt.x;
        cloud->points[i].y = pt.y;
        cloud->points[i].z = pt.z;
        cloud->points[i].intensity = static_cast<float>(pt.intensity); // Convert intensity to float
    }

    // Set the width, height, and is_dense properties
    cloud->width = cloud->points.size();
    cloud->height = 1; // Point clouds are unordered by default
    cloud->is_dense = true;

    return cloud;
}

