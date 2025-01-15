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
    using Element = std::pair<double, T>;

    // Constructors
    StampedRingBuffer() = default;
    StampedRingBuffer(size_t capacity);

    StampedRingBufferIterator<T> begin();
    StampedRingBufferIterator<T> end();
    StampedRingBufferIterator<T> iteratorLowerBound(double ts);

    void addElement(const Element& new_element);

    const Element* first() const;
    const Element* last() const;

    void removeElementsBefore(double threshold);

    size_t size() const;
    size_t capacity() const;
    bool isFull() const;
    bool isEmpty() const;

private:
    size_t capacity_ = 0;  
    size_t size_ = 0;  
    int head_ = 0;  
    std::vector<Element> buffer_;

    void increment(int& idx) const;
    void decrement(int& idx) const;

    int idxAdd(int idx, int offset) const;
    int idxDiff(int idx0, int idx1) const;
    int idxLowerBound(double ts) const;
    int getTail() const;
};

// Define a custom iterator
template <typename T>
class StampedRingBufferIterator {
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = typename StampedRingBuffer<T>::Element;
    using difference_type = ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;

    StampedRingBufferIterator(std::vector<typename StampedRingBuffer<T>::Element>& buffer, size_t capacity, int index)
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
    std::vector<typename StampedRingBuffer<T>::Element>& buffer_;
    size_t capacity_;
    int index_;
};

#include "StampedRingBuffer.tpp" // Include template definitions