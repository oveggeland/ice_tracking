/*
Templated container for a ringbuffer, with a timestamp key for efficient lookup.
*/

#pragma once

#include <vector>
#include <cassert>
#include <iterator>
#include <algorithm>
#include <cstdint>

// Forward declaration of the iterator
template <typename T>
class StampedRingBufferIterator;

template <typename T>
class StampedRingBuffer {
public:
    // Constructors
    StampedRingBuffer() = default;
    StampedRingBuffer(size_t capacity);

    // Iterators TODO: Make const?
    const StampedRingBufferIterator<T> begin() const;
    const StampedRingBufferIterator<T> end() const;
    const StampedRingBufferIterator<T> iteratorLowerBound(double ts) const;

    void addPoint(const T& point);

    int size(){
        return size_;
    }

private:
    size_t capacity_ = 0;  
    size_t size_ = 0;  
    int head_ = 0;
    std::vector<T> buffer_;

    void increment(int& idx) const;

    int idxFirst() const;
    int idxLast() const;
    int idxAdd(int idx, int offset) const;
    int idxDiff(int idx0, int idx1) const;

    int idxLowerBound(double ts) const;
};

// Define a custom iterator
template <typename T>
class StampedRingBufferIterator {
public:
    StampedRingBufferIterator(const std::vector<T>& buffer, const size_t capacity, int index)
        : buffer_(buffer), capacity_(capacity), index_(index) {}

    const T& operator*() { return buffer_[index_]; }
    const T* operator->() { return &buffer_[index_]; }

    StampedRingBufferIterator& operator++() {
        if (++index_ >= capacity_)
            index_ = 0;
        return *this;
    }

    bool operator==(const StampedRingBufferIterator& other) const {
        return index_ == other.index_;
    }

    bool operator!=(const StampedRingBufferIterator& other) const {
        return !(*this == other);
    }

    int distance_to(const StampedRingBufferIterator& other) const {
        if (other.index_ >= index_) 
            return other.index_ - index_;
        else
            return capacity_ - index_ + other.index_;
    }

private:
    const std::vector<T>& buffer_;
    const size_t capacity_;
    int index_;
};

#include "StampedRingBuffer.tpp" // Include template definitions