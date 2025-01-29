#pragma once

#include <vector>

#include <vector>
#include <tuple>
#include <limits>
#include <cmath>

#include <open3d/Open3D.h>
#include <open3d/t/geometry/PointCloud.h>

template <typename T>
std::tuple<double, double, T, T, double> estimateMoments(const std::vector<T>& vec) {
    double sum = 0;
    double sum2 = 0;
    double sum3 = 0;  // For the third central moment
    T min = std::numeric_limits<T>::max();  // Initialize min to the largest possible value
    T max = std::numeric_limits<T>::min();  // Initialize max to the smallest possible value

    // Loop through the vector to calculate sum, sum of squares, min, and max
    for (const auto& p : vec) {
        sum += p;
        sum2 += p * p;

        // Update min and max
        if (p < min) {
            min = p;
        }
        if (p > max) {
            max = p;
        }
    }

    // Calculate mean
    double mean = sum / vec.size();

    // Second loop to compute the third central moment (skewness numerator)
    for (const auto& p : vec) {
        double deviation = p - mean;
        sum3 += deviation * deviation * deviation;
    }

    // Calculate variance
    double variance = sum2 / vec.size() - (mean * mean);

    // Calculate skewness (normalized third central moment)
    double skewness = (sum3 / vec.size()) / std::pow(std::sqrt(variance), 3);

    return {mean, variance, skewness, min, max};
}



/**
 * Calculate percentile. As a side effect, the function also sorts the vector by numerical value.
 */
template <typename T>
T calculatePercentile(std::vector<T>& data, double p) {
    // Sort the data first
    std::sort(data.begin(), data.end());

    // Calculate the rank (index) corresponding to the desired percentile
    size_t idx = p * (data.size() - 1);
   
    return data[idx];
}


// Function to calculate the histogram of data
template <typename T>
std::vector<int> calculateHistogram(const std::vector<T>& data, int num_bins, T min, T max) {
    T bin_width = (max - min) / num_bins;
    std::vector<int> histogram(num_bins, 0);

    // Populate the histogram
    for (T value : data) {
        int bin = static_cast<int>((value - min) / bin_width);
        if (bin >= num_bins) bin = num_bins - 1;  // Handle edge case
        histogram[bin]++;
    }

    return histogram;
}

// Function to find the "peak" value of the histogram
template <typename T>
T findHistogramPeak(const std::vector<T>& data, int num_bins, T min, T max) {
    // Make histogram
    std::vector<int> histogram = calculateHistogram(data, num_bins, min, max);

    // Find the bin with the maximum count
    int peak_bin = std::distance(histogram.begin(), std::max_element(histogram.begin(), histogram.end()));

    // Return the midpoint of the peak bin
    T bin_width = (max - min) / num_bins;
    return min + peak_bin * bin_width + bin_width / 2;
}