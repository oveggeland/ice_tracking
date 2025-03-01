#include "AugmentedImageFrame.h"

AugmentedImageFrame::AugmentedImageFrame(const double ts, const cv::Mat& img, const open3d::t::geometry::PointCloud& pcd, const Camera& camera, const double scale) : ts_(ts) {
    if (scale != 1.0)
        resize(img, img_, cv::Size(), scale, scale);
    else
        img_ = img;

    int num_points = pcd.IsEmpty()? 0: pcd.GetPointPositions().GetShape(0);
    if (num_points == 0)
        return;

    // Map to world coords
    float* position_ptr = pcd.GetPointPositions().Contiguous().GetDataPtr<float>();
    Eigen::Matrix3Xf r_world = Eigen::Map<Eigen::Matrix3Xf>(position_ptr, 3, num_points);

    // Attribute: elevation
    elevation_ = -r_world.row(2);

    // Attribute: intensity
    if (pcd.HasPointAttr("intensities")){
        float* intensity_ptr = pcd.GetPointAttr("intensities").Contiguous().GetDataPtr<float>();
        intensity_ = Eigen::Map<Eigen::VectorXf>(intensity_ptr, num_points);
    }
    else{
        ROS_WARN("AugmentedImageFrame - intensities not available");
        intensity_.resize(num_points);
    }

    // Attribute: deformation
    if (pcd.HasPointAttr("deformation")){
        float* deformation_ptr = pcd.GetPointAttr("deformation").Contiguous().GetDataPtr<float>();
        deformation_ = Eigen::Map<Eigen::VectorXf>(deformation_ptr, num_points);
    }
    else
        deformation_.resize(num_points); // Don't care about the content


    // Project and find inliers
    uv_ = camera.projectFromWorld(r_world, false); // TODO: The points should be distorted
    inliers_ = camera.getInliers(uv_);

    if (scale != 1.0)
        uv_ = uv_.array() * scale;
}

// Crop to [0, 255] and apply colormap. Clip values to min and max if provided.
cv::Mat getColorMap(const Eigen::VectorXf& values, int color_map, float min = NAN, float max = NAN) {
    // Use provided min/max if valid, otherwise compute from values
    float min_val = std::isnan(min) ? values.minCoeff() : min;
    float max_val = std::isnan(max) ? values.maxCoeff() : max;

    // Prevent division by zero (if all values are the same)
    if (max_val == min_val) {
        max_val += 1.0f;
    }

    // Normalize values to [0, 1] within the given range
    Eigen::VectorXf normalized = ((values.array() - min_val) / (max_val - min_val))
                                    .cwiseMax(0.0f).cwiseMin(1.0f);

    // Convert to OpenCV grayscale format [0, 255]
    cv::Mat grayscale(1, values.size(), CV_8UC1);
    for (int i = 0; i < values.size(); ++i) {
        grayscale.at<uchar>(0, i) = static_cast<uchar>(normalized(i) * 255.0f);
    }

    // Apply colormap
    cv::Mat colored;
    cv::applyColorMap(grayscale, colored, color_map);
    
    return colored;
}


// Constant color
cv::Mat AugmentedImageFrame::getImposedImage(const cv::Scalar c) const{ 
    cv::Mat imposed = img_.clone();

    for (const int& idx: inliers_){
        cv::circle(imposed, cv::Point2f(uv_(0, idx), uv_(1, idx)), 3, c, -1);
    }

    return imposed;
}

void drawPoint(cv::Mat& img, const Eigen::Vector2f uv, const cv::Vec3b& color, int size = 3) {
    // Convert Eigen::Vector2f to cv::Point
    cv::Point center(uv(0), uv(1));

    // Calculate the top-left corner of the square
    float half_size = size / 2;
    cv::Point top_left(center.x - half_size, center.y - half_size);
    cv::Point bottom_right(center.x + half_size, center.y + half_size);

    // Draw the square on the image
    cv::rectangle(img, top_left, bottom_right, color, -1);  // -1 means filled square
}


// Point-wise color
cv::Mat AugmentedImageFrame::getImposedImage(const cv::Mat& colors) const {
    cv::Mat imposed = img_.clone();

    for (const int& idx: inliers_){
        drawPoint(imposed, uv_.col(idx), colors.at<cv::Vec3b>(0, idx), 5);
    }

    return imposed;
}

// From values
cv::Mat AugmentedImageFrame::getImposedImage(const Eigen::VectorXf& values, int color_map, float min, float max) const {
    if (pointCount() < 1)
        return img_.clone();
    cv::Mat colors = getColorMap(values, color_map, min, max);
    return getImposedImage(colors);
}