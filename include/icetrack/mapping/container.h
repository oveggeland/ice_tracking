#pragma once

#include <stdint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



#include <iterator>

struct point {
    double x;
    double y;
    double z;
    uint8_t intensity;
    double ts;
};

// Template iterator to access any field of point
template <typename Field>
class FieldIterator {
public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = typename std::remove_reference<Field>::type; // Value type of the field
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;

    FieldIterator(std::vector<point>* points, std::size_t index) 
        : points_(points), index_(index) {}

    reference operator*() {
        return get_field_at(index_);
    }

    pointer operator->() {
        return &get_field_at(index_);
    }

    FieldIterator& operator++() {
        ++index_;
        return *this;
    }

    FieldIterator operator++(int) {
        FieldIterator tmp = *this;
        ++(*this);
        return tmp;
    }

    bool operator==(const FieldIterator& other) const {
        return index_ == other.index_;
    }

    bool operator!=(const FieldIterator& other) const {
        return !(*this == other);
    }

    difference_type operator-(const FieldIterator& other) const {
        return index_ - other.index_;
    }

    FieldIterator operator+(difference_type n) const {
        return FieldIterator(points_, index_ + n);
    }

    FieldIterator operator-(difference_type n) const {
        return FieldIterator(points_, index_ - n);
    }

private:
    std::vector<point>* points_;
    std::size_t index_;

    // Get the value of the desired field at the current index
    auto& get_field_at(std::size_t index) {
        if constexpr (std::is_same_v<Field, double>) {
            return (*points_)[index].z; // Field is z
        } else if constexpr (std::is_same_v<Field, uint8_t>) {
            return (*points_)[index].intensity; // Field is intensity
        } else if constexpr (std::is_same_v<Field, double>) {
            return (*points_)[index].x; // Field is x
        } else if constexpr (std::is_same_v<Field, double>) {
            return (*points_)[index].y; // Field is y
        } else {
            //static_assert(false, "Unsupported field type");
        }
    }
};

// Function to create an iterator to a specific field
template <typename Field>
FieldIterator<Field> make_field_iterator(std::vector<point>& points, std::size_t index) {
    return FieldIterator<Field>(&points, index);
}


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
