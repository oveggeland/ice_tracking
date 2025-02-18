#include "visualization/ImageGenerator.h"

ImageGenerator::ImageGenerator(ros::NodeHandle& nh, const LidarFrontEnd& lidar_front_end, const PoseGraph& pose_graph)
    : frame_buffer_(lidar_front_end.frameBuffer()), pose_graph_(pose_graph),
      camera_(getParamOrThrow<std::string>(nh, "int_file")) {
    bTc_ = bTc(getParamOrThrow<std::string>(nh, "ext_file"));
    getParamOrThrow<bool>(nh, "/image_generator/generate_dataset", generate_dataset_);
    getParamOrThrow<bool>(nh, "/image_generator/display", display_);
    getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
    getParamOrThrow<double>(nh, "/image_generator/offset", offset_);
    std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
    image_path_ = joinPath(outpath, "dataset/images/");
    bin_path_ = joinPath(outpath, "dataset/projections/");
    makePath(image_path_, true);
    makePath(bin_path_, true);
}

void ImageGenerator::drawPoints(cv::Mat& img, const Eigen::Matrix2Xf& uv, const std::vector<bool> inliers) const{
    cv::Scalar color(0, 255, 0);

    for (int i = 0; i < uv.cols(); ++i) {
        if (inliers[i])
            cv::circle(img, cv::Point2f(uv(0, i), uv(1, i)), 2, color, -1);
    }
}

bool ImageGenerator::getInverseCameraPose(double ts, Eigen::Matrix4f& cTw) const{
    gtsam::Pose3 wTb;
    if (!pose_graph_.poseQuery(ts, wTb)) {
        ROS_WARN_STREAM("Pose query failed at: " << std::fixed << ts);
        return false;
    }
    cTw = wTb.compose(bTc_).inverse().matrix().cast<float>();
    return true;
}

std::vector<bool> ImageGenerator::getInliers(const Eigen::Matrix3Xf& r_cam, const Eigen::Matrix2Xf& uv) const {
    std::vector<bool> inliers;
    inliers.resize(uv.cols());

    for (int i = 0; i < uv.cols(); ++i) {
        if (camera_.inBounds(uv.col(i)) && r_cam(2, i) > 0)
            inliers[i] = true;
        else
            inliers[i] = false;
    }
    return inliers;
}

void ImageGenerator::processImage(double t_img, cv::Mat& img) const{
    // Query pose
    Eigen::Matrix4f cTw;
    if (!getInverseCameraPose(t_img, cTw))
        return;

    // Query points
    double t0 = t_img - offset_;
    double t1 = t_img + offset_;
    CloudFrame::Ptr frame = frame_buffer_.getPoints(t0, t1, false, true, true, true);

    // Project points
    const Eigen::Matrix3Xf r_world = frame->global();
    const Eigen::Matrix3Xf r_cam = (cTw.topLeftCorner<3, 3>() * r_world).colwise() + cTw.topRightCorner<3, 1>();
    const Eigen::Matrix2Xf uv = camera_.projectPoints(r_cam, false);

    // Find inliers
    const std::vector<bool> inliers = getInliers(r_cam, uv);


    // Draw and display
    if (display_){
        drawPoints(img, uv, inliers);
        display("Projected", img);
    }
}

void ImageGenerator::checkBuffer() {
    for (auto it = image_buffer_.begin(); it != image_buffer_.end();) {
        double t_img = it->first;
        if (ros::Time::now().toSec() - t_img > delay_) {
            processImage(t_img, it->second);
            it = image_buffer_.erase(it);
        }
        break;
    }
}

void ImageGenerator::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    double ts = msg->header.stamp.toSec();
    cv::Mat img = camera_.getDistortedImage(msg);
    image_buffer_[ts] = img;

    checkBuffer();
}

void ImageGenerator::display(const std::string& name, const cv::Mat& img) const{
    cv::imshow(name, img);
    cv::waitKey(1);
}
