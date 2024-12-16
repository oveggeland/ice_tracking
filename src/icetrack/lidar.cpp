#include "icetrack/lidar.h"

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

    point_buffer_ = PointCloudBuffer(cloud_interval_ / point_interval_); // Allocate enough memory for a full sliding window

    seg_.setModelType(pcl::SACMODEL_PLANE); // Set the model you want to fit
    seg_.setMethodType(pcl::SAC_RANSAC);    // Use RANSAC to estimate the plane
    seg_.setDistanceThreshold(0.5);        // Set a distance threshold for points to be considered inliers)
    seg_.setProbability(0.99);
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
    double ts_point = msg->header.stamp.toSec(); // Header stamp is valid for the first point
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        if (it[0] > min_x_distance_){
            // Emplace writing to buffer
            PointXYZIT* p = point_buffer_.addPoint();
            
            // Update the other fields
            p->intensity = it[3];
            p->ts = ts_point;

            // Transform the point and directly update the ring buffer
            *reinterpret_cast<Point3*>(p) = bTl_.transformFrom(Point3(it[0], it[1], it[2]));
        }

        ts_point += point_interval_;
    }

    point_buffer_.removePointsBefore(ts_point - cloud_interval_);
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
    auto cloud_ptr = point_buffer_.getPointsWithin(t0, t1);
    if (segmentPlane(cloud_ptr)){
        return true;
    };

    return false;
}

void LidarHandle::init(double ts){
    if (generatePlane(ts))
        init_ = true;
}