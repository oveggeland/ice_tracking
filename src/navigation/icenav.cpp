#include "icetrack/navigation/icenav.h"


// Constructor
IceNav::IceNav(){
    graph_ = NonlinearFactorGraph();
    values_ = Values();

    // Add whatever initialization is required
    gnss_handle_ = GnssHandle();
    imu_handle_ = ImuHandle();
}


void IceNav::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if (finished_) return;

    if (init_){
        imu_handle_.integrate(msg);
    }
    else{
        imu_handle_.init(msg);
    }
}

void IceNav::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (finished_) return;

    // Retrieve timestamp and position
    double ts = msg->header.stamp.toSec();
    gtsam::Point2 xy = gnss_handle_.getMeasurement(msg);

    if (init_){
        ROS_INFO_STREAM("GNSS correction " << correction_count_ << ": " << std::fixed << xy[0] << ", " << xy[1]);
        // Add GNSS factor
        auto gnss_factor = gnss_handle_.getCorrectionFactor(xy, correction_count_);
        graph_.add(gnss_factor);

        // Add IMU integration factor
        auto imu_factor = imu_handle_.finishIntegration(ts, correction_count_);
        graph_.add(imu_factor);

        // TODO: Add altitude constraint
        auto altitude_factor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, 0.2));
        graph_.add(altitude_factor);

        // Add new initial estimates
        Pose3 pose0 = values_.at<Pose3>(X(correction_count_-1));
        Vector3 v0 = values_.at<Vector3>(V(correction_count_-1));
        imuBias::ConstantBias bias0 = values_.at<imuBias::ConstantBias>(B(correction_count_-1));
        NavState state_pred = imu_handle_.predict(NavState(pose0, v0), bias0);
        
        // Insert predictions
        values_.insert(X(correction_count_), state_pred.pose());
        values_.insert(V(correction_count_), state_pred.velocity());
        values_.insert(B(correction_count_), bias0);

        // Optional optimization (every now and then)
        if (correction_count_ < 50 || correction_count_ % 50 == 0){
            LevenbergMarquardtOptimizer optimizer(graph_, values_);
            values_ = optimizer.optimize();
        }

        if (correction_count_ == 3000){
            finish();
        }

        // Reset IMU preintegration
        imu_handle_.resetIntegration(ts, values_.at<imuBias::ConstantBias>(B(correction_count_)));

        // Control parameters
        correction_stamps_.push_back(ts);
        correction_count_ ++;
    }
    else if (imu_handle_.isInit()){
        // Add GNSS factor
        auto gnss_factor = gnss_handle_.getCorrectionFactor(xy, 0);
        graph_.add(gnss_factor);

        // Add altitude factor
        auto altitude_factor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, 0.2));
        graph_.add(altitude_factor);

        // Bias prior
        graph_.addPrior(B(0), imuBias::ConstantBias(), noiseModel::Isotropic::Sigma(6, 0.1));
        graph_.addPrior(V(0), Point3(), noiseModel::Isotropic::Sigma(3, 2));

        // Add initial values
        values_.insert(X(0), Pose3(imu_handle_.getPriorRot(), Point3(xy[0], xy[1], 0)));
        values_.insert(V(0), Point3());
        values_.insert(B(0), imuBias::ConstantBias());
        values_.insert(L(0), Point3());

        // Reset IMU preintegration
        imu_handle_.resetIntegration(ts, imuBias::ConstantBias());

        // Control parameters
        correction_stamps_.push_back(ts);
        correction_count_ = 1;
        init_ = true;
    }
}

void IceNav::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // TODO (Optional)
    // Add lever arm constraint
}




void IceNav::finish(){
    std::ofstream f("/home/oskar/icetrack/output/nav.csv");

    f << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz";
    f << std::endl << std::fixed; 

    for (int i = 0; i < correction_count_; i++){
        Pose3 pose = values_.at<Pose3>(X(i));
        Vector3 x = pose.translation();
        Vector3 ypr = pose.rotation().ypr();
        Vector3 v = values_.at<Vector3>(V(i));
        Vector6 b = values_.at<imuBias::ConstantBias>(B(i)).vector();

        f << correction_stamps_[i] << ",";
        f << x[0] << "," << x[1] << "," << x[2] << ",";
        f << v[0] << "," << v[1] << "," << v[2] << ",";
        f << ypr[2] << "," << ypr[1] << "," << ypr[0] << ",";
        f << b[0] << "," << b[1] << "," << b[2] << "," << b[3] << "," << b[4] << "," << b[5];
        f << std::endl;

    }

    finished_ = true;
}