#include "ImageFrame.h"

// Crop to [0, 255] and apply colormap. Clip values to min and max if provided.
cv::Mat getColorMap(const std::vector<float>& values, int color_map, float min, float max) {
    if (values.empty()) {
        return cv::Mat();  // Return empty matrix if no data
    }

    // Prevent division by zero (if min == max)
    if (max == min) {
        max += 1.0f;
    }

    // Create grayscale matrix (1-row image)
    cv::Mat grayscale(1, static_cast<int>(values.size()), CV_8UC1);

    // Normalize and scale values to [0, 255]
    float range = max - min;
    for (size_t i = 0; i < values.size(); ++i) {
        float normalized = (values[i] - min) / range;
        normalized = std::clamp(normalized, 0.0f, 1.0f);  // Clamp to [0,1]
        grayscale.at<uchar>(0, static_cast<int>(i)) = static_cast<uchar>(normalized * 255.0f);
    }

    // Apply colormap
    cv::Mat colored;
    cv::applyColorMap(grayscale, colored, color_map);

    return colored;
}


// Constant color
cv::Mat ImageFrame::getImposedImage(const cv::Scalar& c) const{ 
    cv::Mat imposed = img_.clone();

    for (const Eigen::Vector2f& uv: uv_){
        imposed.at<cv::Vec3b>(uv(1), uv(0)) = cv::Vec3b(c[0], c[1], c[2]); 
    }

    return imposed;
}

// Point-wise color
cv::Mat ImageFrame::getImposedImage(const cv::Mat& colors) const {
    cv::Mat imposed = img_.clone();

    for (int i = 0; i < uv_.size(); ++i){
        const Eigen::Vector2f& uv = uv_[i];
        imposed.at<cv::Vec3b>(uv(1), uv(0)) = colors.at<cv::Vec3b>(0, i);  // Assign color
    }
   
    return imposed;
}

// From values
cv::Mat ImageFrame::getImposedImage(const std::vector<float>& values, int color_map, float v_min, float v_max) const {
    if (values.empty())
        return img_.clone();

    cv::Mat colors = getColorMap(values, color_map, v_min, v_max);
    return getImposedImage(colors);
}

cv::Mat ImageFrame::getImposedElevationImage(int color_map, float v_min, float v_max) const {
    return getImposedImage(elevation_, color_map, v_min, v_max);
}

cv::Mat ImageFrame::getImposedIntensityImage(int color_map, float v_min, float v_max) const {
    return getImposedImage(intensity_, color_map, v_min, v_max);
}