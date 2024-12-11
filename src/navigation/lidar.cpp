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


bool LidarHandle::newFrame(sensor_msgs::PointCloud2::ConstPtr msg){
    double t_msg = msg->header.stamp.toSec();

    // Check if time for new update
    if (t_msg - ts_ > measurement_interval_){
        // Try plane segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = msgToCloud(msg);
        Vector4 plane_coeffs;
        if (segmentPlane(cloud, plane_coeffs)){
            ts_ = t_msg;
            planeUpdate(plane_coeffs);

            return true;
        };
    }
    return false;
}

boost::shared_ptr<gtsam::NonlinearFactor> LidarHandle::getAltitudeFactor(Key key){
    return boost::make_shared<AltitudeFactor>(key, z_, noiseModel::Isotropic::Sigma(1, 1));
}

boost::shared_ptr<gtsam::NonlinearFactor> LidarHandle::getAttitudeFactor(Key key){
    return boost::make_shared<Pose3AttitudeFactor>(key, Unit3(0, 0, 1), noiseModel::Isotropic::Sigma(2, 0.1), bZ_);
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
        planeUpdate(plane_coeffs);

        init_ = true;
    }
}


void LidarHandle::planeUpdate(Vector4 coeffs){
    z_ = -abs(coeffs[3]);

    bZ_ = Unit3(coeffs.head<3>());
    if (bTl_.rotation().inverse().rotate(bZ_).point3().x() < 0){ // Assert normal vector has positive x in LiDAR frame (pointing down in world frame)
        bZ_ = Unit3(-bZ_.point3());
    }
}