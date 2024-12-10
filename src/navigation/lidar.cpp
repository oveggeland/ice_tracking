#include "icetrack/navigation/lidar.h"

// Read lidar to body transformation
Pose3 readExt(const std::string& filename){
    YAML::Node config = YAML::LoadFile(filename);

    gtsam::Matrix4 Tcb;
    auto Tcb_node = config["T_cam_imu"];

    gtsam::Matrix4 Tlc;
    auto Tlc_node = config["T_lidar_cam"];
    
    
    // Fill the matrix with the values from the YAML node
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            Tcb(i, j) = Tcb_node[i][j].as<double>();
            Tlc(i, j) = Tlc_node[i][j].as<double>();
        }
    }

    gtsam::Pose3 pose3cb(Tcb);
    gtsam::Pose3 pose3lc(Tlc);

    return pose3cb.inverse().compose(pose3lc.inverse());
}


LidarHandle::LidarHandle(){
    bTl_ = readExt("/home/oskar/smooth_sailing/src/smooth_sailing/cfg/calib/ext_right.yaml");

    measurement_interval_ = 5;
    measurement_sigma_ =  1;
    min_x_distance_ = 5;
    min_inlier_count_ = 100;


    double ransac_threshold_ = 0.5;
    double ransac_prob_ = 0.99;

    seg_.setModelType(pcl::SACMODEL_PLANE); // Set the model you want to fit
    seg_.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC to estimate the plane
    seg_.setDistanceThreshold(ransac_threshold_);        // Set a distance threshold for points to be considered inliers)
    seg_.setProbability(ransac_prob_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr LidarHandle::msgToCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert from sensor_msgs/PointCloud2 to PCL point cloud
    pcl::fromROSMsg(*msg, *cloud);

    // Create a new point cloud to store filtered and transformed data
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Get the rotation (R) and translation (t) from the gtsam::Pose3 object
    gtsam::Matrix3 R = bTl_.rotation().matrix();  // 3x3 rotation matrix
    gtsam::Point3 t = bTl_.translation();         // Translation vector

    // Iterate over points, transform and filter points with x > 10
    for (const auto& point : cloud->points) {
        if (point.x < min_x_distance_) {
            continue;
        }

        // Apply the transformation to the point
        Eigen::Vector3d p(point.x, point.y, point.z);  // Original point in Eigen form
        Eigen::Vector3d p_transformed = R * p + Eigen::Vector3d(t.x(), t.y(), t.z());  // Transformed point

        pcl::PointXYZ new_point;
        new_point.x = p_transformed.x();
        new_point.y = p_transformed.y();
        new_point.z = p_transformed.z();
        transformed_cloud->points.push_back(new_point);
    }

    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1; // Unorganized point cloud
    transformed_cloud->is_dense = true;

    return transformed_cloud;
}

bool LidarHandle::segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector4 &plane_coeffs){
    if (cloud->size() < min_inlier_count_){
        return false;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coefficients);

    if (inliers->indices.size() < min_inlier_count_) {
        return false;
    }
    
    for (int i = 0; i < 4; i++)
        plane_coeffs[i] = coefficients->values[i];

    return true;
}

boost::shared_ptr<gtsam::NonlinearFactor> LidarHandle::getCorrectionFactor(sensor_msgs::PointCloud2::ConstPtr msg, Key key, bool &success){
    success = false; // Default to false

    // Check if time for new measurement
    double t_msg = msg->header.stamp.toSec();
    if (t_msg - ts_ < measurement_interval_){ // TODO: Consider the effect of timestamp delays. Stamps are given at start of LiDAR acquisition...
        return nullptr;
    }

    // Try plane segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = msgToCloud(msg);
    Vector4 plane_coeffs;
    if (segmentPlane(cloud, plane_coeffs)){
        ROS_INFO_STREAM(std::fixed << t_msg << ": Altitude measurement of " << plane_coeffs[3]);

        ts_ = t_msg;
        z_ = plane_coeffs[3];
        success=true;
        return boost::make_shared<AltitudeFactor>(key, z_, noiseModel::Isotropic::Sigma(1, 1));
    };

    return nullptr;
}

bool LidarHandle::isInit(){
    return init_;
}

double LidarHandle::getAltitude(){
    return z_;
}

void LidarHandle::init(sensor_msgs::PointCloud2::ConstPtr msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = msgToCloud(msg);
    Vector4 plane_coeffs;
    if (segmentPlane(cloud, plane_coeffs)){
        ts_ = msg->header.stamp.toSec();
        z_ = plane_coeffs[3];
        init_ = true;
    }
}