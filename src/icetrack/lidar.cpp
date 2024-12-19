#include "icetrack/lidar.h"

// Read lidar to body transformation TODO: Generalize and move to other file
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

LidarHandle::LidarHandle(){}

LidarHandle::LidarHandle(ros::NodeHandle nh): nh_(nh){
    std::string ext_file = getParamOrThrow<std::string>(nh_, "/ext_file");
    bTl_ = readExt(ext_file);

    getParamOrThrow(nh_, "/lidar/buffer_period", buffer_period_);
    getParamOrThrow(nh_, "/lidar/point_interval", point_interval_);
    getParamOrThrow(nh_, "/lidar/ransac_threshold", ransac_threshold_);
    getParamOrThrow(nh_, "/lidar/ransac_prob", ransac_prob_);
    getParamOrThrow(nh_, "/lidar/plane_min_inlier_count", min_inlier_count_);

    double min_dist = getParamOrThrow<double>(nh_, "/lidar/min_dist");
    min_dist_square_ = min_dist * min_dist;

    double max_dist = getParamOrThrow<double>(nh_, "/lidar/max_dist");
    max_dist_square_ = max_dist * max_dist;

    getParamOrThrow(nh_, "/lidar/min_intensity", min_intensity_);

    getParamOrThrow(nh_, "/lidar/measurement_interval", measurement_interval_);
    getParamOrThrow(nh_, "/lidar/measurement_sigma", measurement_sigma_);


    point_buffer_ = std::make_shared<PointCloudBuffer>(buffer_period_ / point_interval_); // Allocate a ringbuffer for incoming points

    seg_.setModelType(pcl::SACMODEL_PLANE); // Set the model you want to fit
    seg_.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC to estimate the plane
    seg_.setDistanceThreshold(ransac_threshold_);        // Set a distance threshold for points to be considered inliers)
    seg_.setProbability(ransac_prob_);
}


bool LidarHandle::segmentPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    if (cloud->size() < min_inlier_count_){
        return false;
    }

    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg_.setInputCloud(cloud);
    seg_.segment(*inliers, *coeffs);

    if (inliers->indices.size() < min_inlier_count_) {
        return false;
    }
    // Set some parameters based on segmented plane
    z_ = -abs(coeffs->values[3]);
    bZ_ = Unit3(coeffs->values[0], coeffs->values[1], coeffs->values[2]);
    if (bTl_.rotation().inverse().rotate(bZ_).point3().x() < 0){ // Assert normal vector has positive x in LiDAR frame (pointing down in world frame)
        bZ_ = Unit3(-bZ_.point3());
    }

    return true;
}


/*
Efficient parsing of pointcloud msg. Removing rough outliers on the fly. Emplacement into ringbuffer. 
PointCloud2 message is assumed structured such that each point is (x, y, z, intensity) where all are floating values.
*/
void LidarHandle::addFrame(sensor_msgs::PointCloud2::ConstPtr msg){
    double ts_point = msg->header.stamp.toSec() - point_interval_; // Header stamp is valid for the first point
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        ts_point += point_interval_;

        // Initial rejection tests
        if (
            it[0] == 0.0 ||             // Empty point
            it[3] < min_intensity_ ||   // Outlier
            ts_point <= ts_head_        // Time error
            )
            continue;


        // Assert distance range
        double d2 = it[0]*it[0] + it[1]*it[1] + it[2]*it[2];
        if (d2 < min_dist_square_ || d2 > max_dist_square_)
            continue;

        // Transform to body frame and add to buffer
        Point3 r_body = bTl_.transformFrom(Point3(it[0], it[1], it[2]));

        point_buffer_->addPoint({
            r_body.x(),
            r_body.y(),
            r_body.z(),
            it[3],
            ts_point
        });        
    }
    ts_head_ = ts_point; // Only need to update this at end-of-frame. (All points inside a single frame is sequential)
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

bool LidarHandle::generatePlane(double ts){
    // Timestamp bounds for lidar points to use
    double t0 = ts - 0.5*measurement_interval_;
    double t1 = ts + 0.5*measurement_interval_;

    // Find tail
    auto cloud_ptr = point_buffer_->getPclWithin(t0, t1);
    if (segmentPlane(cloud_ptr)){
        return true;
    };

    return false;
}

void LidarHandle::init(double ts){
    if (generatePlane(ts))
        init_ = true;
}