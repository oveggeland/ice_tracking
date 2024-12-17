#include "icetrack/container.h"

PointCloudBuffer::PointCloudBuffer(size_t capacity)
    : capacity_(capacity), buffer_(capacity), head_(0), size_(0) {}


PointXYZIT* PointCloudBuffer::addPoint(){
    PointXYZIT* p = &buffer_[head_];
    if (++size_ > capacity_)
        size_ = capacity_;
    increment(head_);
    return p;
}

void PointCloudBuffer::addPoint(const PointXYZIT& new_point) {
    buffer_[head_] = new_point;
    if (++size_ > capacity_)
        size_ = capacity_;
    increment(head_);
}

int PointCloudBuffer::idxDiff(int idx0, int idx1) const{
    return (idx1 - idx0 + capacity_) % capacity_;
}

int PointCloudBuffer::idxAdd(int idx, int offset) const{
    return (idx + offset + capacity_) % capacity_;
}

/*
Binary search for lower bound index based on timestamp. Used for efficient look up of specific time intervals.
*/
int PointCloudBuffer::idxLowerBound(double ts) const{    
    int tail = getTail();
    if (buffer_[tail].ts > ts)
        return -1; // Not points match this

    int low = tail;
    int high = idxAdd(head_, -1);
    while (idxDiff(low, high) > 0){
        int mid = (low + idxDiff(low, high)/2 + 1) % capacity_;

        if (buffer_[mid].ts > ts)
            high = idxAdd(mid, -1);
        else
            low = mid;
    } 
    
    assert(ts >= buffer_[high].ts);
    return high;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudBuffer::getPclWithin(double t0, double t1) const{
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (size_ == 0)
        return cloud;

    int idx = idxLowerBound(t0);
    if (idx < 0){
        return cloud;
    }

    cloud->reserve(size_); // There is no way more capacity is needed
    while (idx != head_){
        const PointXYZIT& pt = buffer_[idx];

        if (pt.ts > t1) // We are past interval
            break;
        else if (pt.ts > t0){ // We are in interval, copy points
            cloud->points.emplace_back(); // Adding a dummy element to the back

            pcl::PointXYZI& p_ref = cloud->points.back();
            p_ref.x = pt.x;
            p_ref.y = pt.y;
            p_ref.z = pt.z;
            p_ref.intensity = pt.intensity;
        }

        increment(idx);
    }

    // Set the width, height, and is_dense properties
    cloud->width = cloud->points.size();
    cloud->height = 1; // Point clouds are unordered by default
    cloud->is_dense = true;

    return cloud;
}

std::vector<PointXYZIT> PointCloudBuffer::getPointsWithin(double t0, double t1) const{
    std::vector<PointXYZIT> cloud;
    cloud.reserve(size_);

    int idx = idxLowerBound(t0);
    while (idx != head_){
        const PointXYZIT& pt = buffer_[idx];

        if (pt.ts > t1) // We are past interval
            break;
        else if (pt.ts > t0){ // We are in interval, copy points
            cloud.push_back(pt); // Adding a dummy element to the back
        }

        increment(idx);
    }

    return cloud;
}

void PointCloudBuffer::removePointsBefore(double threshold) {
    int lb = idxLowerBound(threshold);
    if (lb < 0)
        return;
    
    size_ -= (idxDiff(getTail(), lb) + 1);
    assert(size_ >= 0);
}

size_t PointCloudBuffer::size() const {
    return size_;
}

size_t PointCloudBuffer::capacity() const {
    return capacity_;
}

bool PointCloudBuffer::isFull() const {
    return size_ == capacity_;
}

bool PointCloudBuffer::isEmpty() const {
    return size_ == 0;
}

int PointCloudBuffer::getTail() const {
    return (head_ - size_ + capacity_) % capacity_;
}

void PointCloudBuffer::increment(int& idx) const{
    if (++idx >= capacity_)
        idx = 0;
}
void PointCloudBuffer::decrement(int& idx) const{
    if (--idx < 0)
        idx = capacity_ - 1; 
}