#include "StampedRingBuffer.h"

template <typename T>
StampedRingBuffer<T>::StampedRingBuffer(size_t capacity)
    : capacity_(capacity), buffer_(capacity), head_(0), size_(0) {}


template <typename T>
StampedRingBufferIterator<T> StampedRingBuffer<T>::begin(){
    return StampedRingBufferIterator<T>(buffer_, capacity_, idxFirst());
}

template <typename T>
StampedRingBufferIterator<T> StampedRingBuffer<T>::end(){
    return StampedRingBufferIterator<T>(buffer_, capacity_, head_);
}

template <typename T>
StampedRingBufferIterator<T> StampedRingBuffer<T>::iteratorLowerBound(double ts){
    int idx = idxLowerBound(ts);
    return (idx < 0) ? end() : StampedRingBufferIterator<T>(buffer_, capacity_, idx);
}

template <typename T>
void StampedRingBuffer<T>::addPoint(const T& new_point) {
    buffer_[head_] = new_point;
    if (++size_ > capacity_)
        size_ = capacity_;
    increment(head_);
}

template <typename T>
void StampedRingBuffer<T>::increment(int& idx) const {
    if (++idx >= capacity_)
        idx = 0;
}

template <typename T>
int StampedRingBuffer<T>::idxFirst() const {
    int idx = head_ - size_;
    return (idx < 0)? idx + capacity_ : idx;
}

template <typename T>
int StampedRingBuffer<T>::idxLast() const {
    return idxAdd(head_, -1);
}

template <typename T>
int StampedRingBuffer<T>::idxAdd(int idx, int offset) const {
    int ret = idx + offset;
    return (ret >= capacity_) ? ret % capacity_ : ret;
}

template <typename T>
int StampedRingBuffer<T>::idxDiff(int idx0, int idx1) const {
    int ret = idx1 - idx0;
    return (ret < 0) ? ret + capacity_ : ret;
}

template <typename T>
int StampedRingBuffer<T>::idxLowerBound(double ts) const {
    if (size_ == 0 || buffer_[idxLast()].ts < ts)
        return -1;

    int low = idxFirst();
    int high = idxLast();

    while (low != high) {
        int mid = idxAdd(low, idxDiff(low, high) / 2);
        if (buffer_[mid].ts >= ts)
            high = mid;
        else {
            low = mid;
            increment(low);
        }
    }

    return high;
}