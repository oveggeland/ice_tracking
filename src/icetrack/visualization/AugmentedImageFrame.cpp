#include "AugmentedImageFrame.h"

AugmentedImageFrame::AugmentedImageFrame(const double ts, const cv::Mat& img, const open3d::t::geometry::PointCloud& pcd, const Camera& camera) : ts_(ts), img_(img) {
    int num_points = pcd.IsEmpty()? 0: pcd.GetPointPositions().GetShape(0);
    if (num_points == 0)
        return;

    // Map cloud attributes
    float* position_ptr = pcd.GetPointPositions().Contiguous().GetDataPtr<float>();
    Eigen::Matrix3Xf r_world = Eigen::Map<Eigen::Matrix3Xf>(position_ptr, 3, num_points);

    if (pcd.HasPointAttr("intensities")){
        float* intensity_ptr = pcd.GetPointAttr("intensities").Contiguous().GetDataPtr<float>();
        intensity_ = Eigen::Map<Eigen::VectorXf>(intensity_ptr, num_points);
    }

    if (pcd.HasPointAttr("deformation")){
        float* deformation_ptr = pcd.GetPointAttr("deformation").Contiguous().GetDataPtr<float>();
        deformation_ = Eigen::Map<Eigen::VectorXf>(deformation_ptr, num_points);
    }

    elevation_ = -r_world.row(2);

    // Project
    uv_ = camera.projectFromWorld(r_world, false);
    inlier_mask_ = camera.getInlierMask(uv_);
}


void drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers){
    const cv::Scalar color(0, 255, 0);

    for (int i = 0; i < uv.cols(); ++i) {
        if (inliers[i])
            cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, color, -1);
    }
}

// Return image with imposed pointcloud
cv::Mat AugmentedImageFrame::getImposedImage() const {
    cv::Mat img_imposed = img_.clone();

    drawPoints(img_imposed, uv_, inlier_mask_);
    return img_imposed;
}

int AugmentedImageFrame::inlierCount() const {
    int sum = 0;
    for (const bool& is_inlier: inlier_mask_){
        if (is_inlier) ++sum;
    }
    return sum;
}