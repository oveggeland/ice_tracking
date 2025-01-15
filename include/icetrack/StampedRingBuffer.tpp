#include "StampedRingBuffer.h"

template <typename T>
StampedRingBuffer<T>::StampedRingBuffer(size_t capacity)
    : capacity_(capacity), buffer_(capacity), head_(0), size_(0) {}

template <typename T>
void StampedRingBuffer<T>::addElement(const Element& new_element) {
    assert(new_element.first > (size_ > 0 ? last()->first : -std::numeric_limits<double>::infinity()));
    buffer_[head_] = new_element;
    if (++size_ > capacity_)
        size_ = capacity_;
    increment(head_);
}

template <typename T>
const typename StampedRingBuffer<T>::Element* StampedRingBuffer<T>::first() const {
    return &buffer_[getTail()];
}

template <typename T>
const typename StampedRingBuffer<T>::Element* StampedRingBuffer<T>::last() const {
    return &buffer_[idxAdd(head_, -1)];
}

template <typename T>
int StampedRingBuffer<T>::idxDiff(int idx0, int idx1) const {
    return (idx1 - idx0 + capacity_) % capacity_;
}

template <typename T>
int StampedRingBuffer<T>::idxAdd(int idx, int offset) const {
    return (idx + offset + capacity_) % capacity_;
}

template <typename T>
int StampedRingBuffer<T>::idxLowerBound(double ts) const {
    if (size_ == 0 || last()->first < ts)
        return -1;

    int low = getTail();
    int high = idxAdd(head_, -1);

    while (low != high) {
        int mid = idxAdd(low, idxDiff(low, high) / 2);
        if (buffer_[mid].first >= ts)
            high = mid;
        else {
            low = mid;
            increment(low);
        }
    }

    return high;
}


template <typename T>
StampedRingBufferIterator<T> StampedRingBuffer<T>::begin(){
    return StampedRingBufferIterator<T>(buffer_, capacity_, getTail());
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
void StampedRingBuffer<T>::removeElementsBefore(double threshold) {
    int lb = idxLowerBound(threshold);
    if (lb < 0)
        return;
    size_ -= idxDiff(getTail(), lb);
}

template <typename T>
size_t StampedRingBuffer<T>::size() const {
    return size_;
}

template <typename T>
size_t StampedRingBuffer<T>::capacity() const {
    return capacity_;
}

template <typename T>
bool StampedRingBuffer<T>::isFull() const {
    return size_ == capacity_;
}

template <typename T>
bool StampedRingBuffer<T>::isEmpty() const {
    return size_ == 0;
}

template <typename T>
int StampedRingBuffer<T>::getTail() const {
    return (head_ - size_ + capacity_) % capacity_;
}

template <typename T>
void StampedRingBuffer<T>::increment(int& idx) const {
    if (++idx >= capacity_)
        idx = 0;
}

template <typename T>
void StampedRingBuffer<T>::decrement(int& idx) const {
    if (--idx < 0)
        idx = capacity_ - 1;
}
