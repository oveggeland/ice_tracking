#include "icetrack/navigation/icenav.h"


geometry_msgs::Pose convertToRosPose(const gtsam::Pose3& pose) {
    geometry_msgs::Pose ros_pose;

    // Set the position
    ros_pose.position.x = pose.x();
    ros_pose.position.y = pose.y();
    ros_pose.position.z = pose.z();

    // Set the orientation
    gtsam::Quaternion quat = pose.rotation().toQuaternion();
    ros_pose.orientation.x = quat.x();
    ros_pose.orientation.y = quat.y();
    ros_pose.orientation.z = quat.z();
    ros_pose.orientation.w = quat.w();

    return ros_pose;
}

// Constructor
IceNav::IceNav(ros::NodeHandle nh, double lag){
    smoother_ = BatchFixedLagSmoother(lag);

    // Outstream
    f_out_ = std::ofstream("/home/oskar/icetrack/output/nav.csv");
    f_out_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz";
    f_out_ << std::endl << std::fixed; 

    // Pose publisher
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_with_covariance", 10);

    // Sensor handles
    gnss_handle_ = GnssHandle();
    imu_handle_ = ImuHandle();
}

void IceNav::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if (init_){
        imu_handle_.integrate(msg);
    }
    else{
        imu_handle_.init(msg);
    }
}

void IceNav::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    // Retrieve timestamp and position
    double ts = msg->header.stamp.toSec();
    gtsam::Point2 xy = gnss_handle_.getMeasurement(msg);

    if (init_){
        // Add GNSS factor
        auto gnss_factor = gnss_handle_.getCorrectionFactor(xy, correction_count_);
        graph_.add(gnss_factor);

        // Add IMU integration factor
        auto imu_factor = imu_handle_.finishIntegration(ts, correction_count_);
        graph_.add(imu_factor);

        update(ts);
    }
    else if (imu_handle_.isInit()){
        initialize(ts, xy);
    }
}

void IceNav::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // TODO (Optional)
    // Add lever arm constraint
}


Pose3 IceNav::getLastPose(double& t_pose){
    t_pose = prev_ts_;
    return prev_pose_;
}


void IceNav::initialize(double ts, Point2 initial_xy){
    // Initialize instances 
    graph_ = NonlinearFactorGraph();
    values_ = Values();

    graph_.addPrior(B(0), imuBias::ConstantBias(), noiseModel::Isotropic::Sigma(6, 0.1));
    graph_.addPrior(V(0), Point3(), noiseModel::Isotropic::Sigma(3, 2));

    // GNSS factor
    auto gnss_factor = gnss_handle_.getCorrectionFactor(initial_xy, 0);
    graph_.add(gnss_factor);

    // Altitude factor
    auto altitude_factor = AltitudeFactor(X(0), 0, noiseModel::Isotropic::Sigma(1, 2));
    graph_.add(altitude_factor);

    // Add initial values
    prev_ts_ = ts;
    prev_pose_ = Pose3(imu_handle_.getPriorRot(), (Point3() << initial_xy, 0).finished());
    prev_vel_ = Point3();
    prev_bias_ = imuBias::ConstantBias();

    values_.insert(X(0), prev_pose_);
    values_.insert(V(0), prev_vel_);
    values_.insert(B(0), prev_bias_);

    // Fixed lag stuff
    stamps_[X(0)] = ts;
    stamps_[V(0)] = ts;
    stamps_[B(0)] = ts;

    // Reset IMU preintegration
    imu_handle_.resetIntegration(ts, imuBias::ConstantBias());

    // Control parameters
    correction_count_ = 1;
    init_ = true;
}


void IceNav::update(double ts){
    // TODO: Fix Levered Altitude constraint
    auto altitude_factor = AltitudeFactor(X(correction_count_), 0, noiseModel::Isotropic::Sigma(1, 2));
    graph_.add(altitude_factor);

    // Add new initial estimates
    NavState state_pred = imu_handle_.predict(NavState(prev_pose_, prev_vel_), prev_bias_);
    
    // Initial estimates
    values_.insert(X(correction_count_), state_pred.pose());
    values_.insert(V(correction_count_), state_pred.velocity());
    values_.insert(B(correction_count_), prev_bias_);

    // Key timestamps
    stamps_[X(correction_count_)] = ts;
    stamps_[V(correction_count_)] = ts;
    stamps_[B(correction_count_)] = ts;

    // Add recent information to fixed lag smoother
    smoother_.update(graph_, values_, stamps_);
    stamps_.clear();
    values_.clear();
    graph_.resize(0);

    // Update current state
    prev_ts_ = ts;
    prev_pose_ = smoother_.calculateEstimate<Pose3>(X(correction_count_));
    prev_vel_ = smoother_.calculateEstimate<Point3>(V(correction_count_));
    prev_bias_ = smoother_.calculateEstimate<imuBias::ConstantBias>(B(correction_count_));

    // Write to file
    f_out_ << ts << ",";
    f_out_ << prev_pose_.translation()[0] << "," << prev_pose_.translation()[1] << "," << prev_pose_.translation()[2] << ",";
    f_out_ << prev_vel_[0] << "," << prev_vel_[1] << "," << prev_vel_[2] << ",";
    f_out_ << prev_pose_.rotation().ypr()[2] << "," << prev_pose_.rotation().ypr()[1] << "," << prev_pose_.rotation().ypr()[0] << ",";
    f_out_ << prev_bias_.accelerometer()[0] << "," << prev_bias_.accelerometer()[1] << "," << prev_bias_.accelerometer()[2] << ",";
    f_out_ << prev_bias_.gyroscope()[0] << "," << prev_bias_.gyroscope()[1] << "," << prev_bias_.gyroscope()[2];
    f_out_ << std::endl;

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time(ts);
    msg.header.frame_id = "IMU";
    msg.pose.pose = convertToRosPose(prev_pose_);

    pose_pub_.publish(msg);

    // Reset IMU preintegration
    imu_handle_.resetIntegration(ts, prev_bias_);

    // Control parameters
    correction_count_ ++;
}