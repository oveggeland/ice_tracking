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
    StampedRingBufferIterator<T> begin();
    StampedRingBufferIterator<T> end();
    StampedRingBufferIterator<T> iteratorLowerBound(double ts);

    void addPoint(const T& point);

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
    using iterator_category = std::forward_iterator_tag;
    using difference_type = ptrdiff_t;
    using pointer = T*;
    using reference = T&;

    StampedRingBufferIterator(std::vector<T>& buffer, size_t capacity, int index)
        : buffer_(buffer), capacity_(capacity), index_(index) {}

    reference operator*() { return buffer_[index_]; }
    pointer operator->() { return &buffer_[index_]; }

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
    std::vector<T>& buffer_;
    size_t capacity_;
    int index_;
};

#include "StampedRingBuffer.tpp" // Include template definitions