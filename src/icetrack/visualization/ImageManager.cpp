#include "visualization/ImageManager.h"

ImageManager::ImageManager(ros::NodeHandle& nh, const PoseGraph& pose_graph, const CloudManager& cloud_manager)
    : nh_(nh), camera_(nh), output_(nh), pose_graph_(pose_graph), cloud_manager_(cloud_manager){

    // Load config
    getParamOrThrow<bool>(nh, "/image_generator/enabled", enabled_);
    getParamOrThrow<bool>(nh, "/image_generator/display", display_);
    getParamOrThrow<double>(nh, "/image_generator/delay", delay_);
    getParamOrThrow<double>(nh, "/image_generator/offset", offset_);

    // Set visualizer from camera intrinsics
    CameraIntrinsics intrinsics = camera_.intrinsics();
    double w = intrinsics.w / 2.0;
    double h = intrinsics.h / 2.0;
    pinhole_params_.intrinsic_. SetIntrinsics(w, h, w/2, w/2, w/2 - 0.5, h/2 - 0.5);

    visualizer_.CreateVisualizerWindow("Offscreen Render", intrinsics.w, intrinsics.h, 0, 0, false);
}


void ImageManager::display(const std::string& window_name, const cv::Mat& img) const {
    cv::imshow(window_name, img);

    // Wait for a key indefinitely if 'p' (ASCII 112) is pressed
    char key = cv::waitKey(1);  // Non-blocking wait
    if (key == 'p' || key == ' ') {  // Pause on 'p' or spacebar
        while (cv::waitKey(0) < 0);  // Wait indefinitely until any key is pressed
    }
    else if (key == 'q')
        ros::shutdown();
}


void ImageManager::processImage(double t_img, const cv::Mat& img){
    timer_buffer_.pop_front();

    display("Raw image", img);
    return;

    // // Query pose (world to camera transformation)
    // gtsam::Pose3 wTb;
    // if (!pose_graph_.poseQuery(t_img, wTb)){
    //     ROS_ERROR_STREAM("ImageManager::processImage - Could not get pose at " << std::fixed << t_img);
    //     return;
    // }
    // camera_.updateTransform(wTb);

    // // Get cloud
    // std::shared_ptr<open3d::geometry::PointCloud> cloud = cloud_manager_.getCloud();
    // if (cloud->points_.size() == 0)
    //     return;

    // // Create a visualizer


    // // Update visualizer with new cloud
    // visualizer_.ClearGeometries();
    // visualizer_.AddGeometry(cloud);
    // visualizer_.UpdateGeometry(cloud);
    // visualizer_.UpdateRender();

    // // Set camera parameters using pinhole model
    // pinhole_params_.extrinsic_ = camera_.getPose().cast<double>();
    // visualizer_.GetViewControl().ConvertFromPinholeCameraParameters(pinhole_params_, true);

    // // Capture rendered image
    // auto render_image = visualizer_.CaptureScreenFloatBuffer();

    // // Convert Open3D image to OpenCV format
    // cv::Mat cv_image(render_image->height_, render_image->width_, CV_32FC3, render_image->data_.data());
    // cv_image.convertTo(cv_image, CV_8UC3, 255.0);
    // cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
    
    // display("test", cv_image);
    // Process the generated image (e.g., save or publish it)
    //cv::imwrite("output.png", cv_image);
    // Create a visualizer with an offscreen renderer
    //open3d::visualization::rendering::OffscreenRenderer renderer(640, 480);
    
    // // Create a scene and add the cloud
    // open3d::visualization::rendering::Scene scene;
    // scene.AddGeometry("point_cloud", cloud, nullptr);

    // // Convert wTb to Open3D camera parameters
    // Eigen::Matrix4d cam_transform = camera_.getPose().cast<double>();
    // open3d::visualization::rendering::Camera camera;
    // camera.SetModelMatrix(cam_transform);

    // // Set the camera view
    // renderer.SetCamera(camera);

    // // Render the scene to an image
    // auto rendered_image = renderer.RenderToImage();

    // // Convert Open3D image to OpenCV format
    // cv::Mat cv_image(rendered_image->height_, rendered_image->width_, CV_8UC3, rendered_image->data_.data());
    // cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
    
    // display("test", cv_image);
    /*
    TODO:
    - Create visualizer object, 
    - Add cloud
    - Set camera view
    - Generate image from view (in opencv format)
    */



    // Query cloud (in world frame)
    // const auto cloud = cloud_manager_.cloudQuery(false, t_img - offset_, t_img + offset_);
    // int num_points = cloud.IsEmpty()? 0: cloud.GetPointPositions().GetShape(0);
    // if (num_points == 0){
    //     ROS_WARN_STREAM("ImageManager::processImage - No cloud points available at: " << std::fixed << t_img);
    // }

    // Generate augmented image
    // ImageFrame img_frame(t_img, img, cloud, camera_);
    
    // auto img_imposed = img_frame.getImposedImage();
    // display("imposed", img_imposed);
    //output_.newImageFrame(img_frame);
}

// Main entry point. Unpack the image and schedule processing. 
void ImageManager::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    if (!enabled_)
        return;

    // Get timestamp and image
    double ts = msg->header.stamp.toSec();
    cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

    // Schedule processing
    timer_buffer_.push_back(nh_.createTimer(
        ros::Duration(delay_), // Duration
        boost::bind(&ImageManager::processImage, this, ts, img), // Callback
        true // One-shot
    ));
}