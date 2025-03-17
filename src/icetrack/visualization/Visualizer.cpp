#include "visualization/Visualizer.h"

Visualizer::Visualizer(ros::NodeHandle& nh) : camera_(nh){

    // Assert enabled
    bool enabled = getParamOrThrow<bool>(nh, "/visualizer/enabled");
    if (!enabled)
        return;

    // Load config
    getParamOrThrow<bool>(nh, "/visualizer/display", display_);
    getParamOrThrow<bool>(nh, "/visualizer/publish", publish_);
    getParamOrThrow<double>(nh, "/visualizer/delay", delay_);

    getParamOrThrow<double>(nh, "/visualizer/scale", scale_);
    frame_ = ImageFrame(scale_);

    // Publisher
    if (publish_){
        std::string topic = getParamOrThrow<std::string>(nh, "/visualizer/topic");
        publisher_ = nh.advertise<sensor_msgs::Image>(topic, 1);
    }

    // Setup subscribers
    std::string cloud_topic = getParamOrThrow<std::string>(nh, "/cloud_topic");
    cloud_sub_ = nh.subscribe(cloud_topic, 1, &Visualizer::cloudCallback, this);

    std::string image_topic = getParamOrThrow<std::string>(nh, "/image_topic");
    image_sub_ = nh.subscribe(image_topic, 1, &Visualizer::imageCallback, this);

    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_sub_ = nh.subscribe(pose_topic, 1, &Visualizer::poseCallback, this);
}


void Visualizer::display(const std::string& window_name, const cv::Mat& img) const {
    cv::imshow(window_name, img);

    // Pause or shut down?
    char key = cv::waitKey(1);  // Non-blocking wait
    if (key == ' ') {  // Pause on spacebar
        while (cv::waitKey(0) != ' ');  // Wait indefinitely until spacebar is pressed
    }
    else if (key == 'q')
        ros::shutdown();
}


void Visualizer::visualize(double t_img, const cv::Mat& img){
    // Update image
    frame_.setImage(img);

    // Find new pose
    gtsam::Pose3 wTb = getPose(t_img);
    camera_.updateTransform(wTb);

    // Add points from cloud
    processLatestCloud();

    // Generate elevation image
    cv::Mat img_elev = frame_.getImposedElevationImage();
    
    // Output
    if (display_)
        display("Raw image", img_elev);
    if (publish_)
        publish(img_elev);
}


void Visualizer::publish(const cv::Mat& img){
    try{
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        publisher_.publish(img_msg);
    }catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge conversion failed: %s", e.what());
    }    
}


void Visualizer::checkImageBuffer(){
    auto it = image_buffer_.begin();

    while (it != image_buffer_.end()) {
        const double t_img = it->first;

        if (t_img > t1_) {
            return;  // Stop processing as all future timestamps are too new
        } else if (t_img >= t0_) {
            visualize(t_img, it->second);  // Process valid images
        }

        // Erase the processed element and get the next iterator safely
        it = image_buffer_.erase(it);
    }
}


// Main entry for image used to visualize the clouds
void Visualizer::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        // Get timestamp and image
        double ts = msg->header.stamp.toSec();
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    
        image_buffer_[ts] = img;
        checkImageBuffer();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge conversion failed: %s", e.what());
    }    
}


void Visualizer::processLatestCloud(){
    if (!cloud_msg_ || cloud_msg_->data.empty()){
        ROS_WARN("Received empty PointCloud2 message.");
        return;
    }

    // Ensure good cloud format
    if (cloud_msg_->point_step != 16){
        ROS_ERROR("PointCloud format is not 4xfloat...");
        return;
    }

    // Allocate memory in frame
    frame_.reset(cloud_msg_->width);

    // Pre-compute transformation 
    const Eigen::Matrix4f& transform = camera_.transform();
    const Eigen::Matrix3f& R = transform.topLeftCorner<3, 3>();
    const Eigen::Vector3f& t = transform.topRightCorner<3, 1>();

    // Iterate over cloud and push to frame if inlier
    sensor_msgs::PointCloud2ConstIterator<PointXYZI> cloud_it(*cloud_msg_, "x");
    for (; cloud_it != cloud_it.end(); ++cloud_it) {
        // Dereference iterator
        const PointXYZI& point = *cloud_it;

        // Get cam frame representation
        const Eigen::Vector3f p_world(point.x, point.y, point.z);
        Eigen::Vector3f p_cam = R * p_world + t;

        if (p_cam.z() < 0)
            continue;   // Behind the camera
        
        Eigen::Vector2f uv = camera_.projectPoint(p_cam, true); // Project and undistort
        if (camera_.inBounds(uv))
            frame_.addPoint(-p_world.z(), point.i, uv);
    }
}


/*
Save shared pointer, for later processing (avoids copying data)
*/
void Visualizer::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    cloud_msg_ = msg;
}

/*
Estimate pose by interpolating the last two poses.
*/
gtsam::Pose3 Visualizer::getPose(const double ts){
    const double alpha = (ts - t0_) / (t1_ - t0_);
    return pose0_.interpolateRt(pose1_, alpha);
}

/*
Reset prev pose to current pose
*/
void Visualizer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    t0_ = t1_;
    pose0_ = pose1_;

    t1_ = msg->header.stamp.toSec();
    pose1_ = poseRosToGtsam(msg->pose);
}