#include "icetrack/navigation/icenav.h"

// Constructor
IceNav::IceNav(ros::NodeHandle nh, double lag): lag_(lag){
    // Initialize instances 
    graph_ = NonlinearFactorGraph();
    values_ = Values();

    LevenbergMarquardtParams p;
    p.maxIterations = 25;
    p.useFixedLambdaFactor = false;
    smoother_ = BatchFixedLagSmoother(lag_, p);

    // Outstream
    f_out_ = std::ofstream("/home/oskar/icetrack/output/nav/nav.csv");
    f_out_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz";
    f_out_ << std::endl << std::fixed; 

    // Sensor handles
    gnss_ = GnssHandle();
    imu_ = ImuHandle();
}


void IceNav::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if (init_){
        imu_.integrate(msg);
    }
    else{
        imu_.init(msg);
    }
}


void IceNav::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (init_){
        auto gnss_factor = gnss_.getCorrectionFactor(msg, X(correction_count_));
        graph_.add(gnss_factor);

        update(msg->header.stamp.toSec());
    }
    else{
        gnss_.init(msg);

        // The system is initialized here when sensors are ready
        if (gnss_.isInit() and imu_.isInit()){
            auto gnss_factor = gnss_.getCorrectionFactor(msg, X(0));
            graph_.add(gnss_factor);

            initialize(msg->header.stamp.toSec());
        }
    } 
}


void IceNav::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // TODO (Optional): Add altitude measurement from lidar
}


void IceNav::initialize(double ts){
    // Initial state
    ts_ = ts;
    pose_ = Pose3(imu_.getPriorRot(), gnss_.getPriorPosition());
    vel_ = gnss_.getPriorVelocity();
    bias_ = imuBias::ConstantBias(); //imuBias::ConstantBias(Point3(-0.03, 0.07, -0.14), Point3(-0.002, 0.002, -0.0024));

    writeToFile(); // Write initial values to file

    // Add prior factors
    graph_.addPrior(B(0), bias_, noiseModel::Isotropic::Sigma(6, 0.1));
    graph_.addPrior(V(0), vel_, noiseModel::Isotropic::Sigma(3, 1));

    auto altitude_factor = AltitudeFactor(X(0), 0, noiseModel::Isotropic::Sigma(1, 2));
    graph_.add(altitude_factor);

    // Add to values_
    values_.insert(X(0), pose_);
    values_.insert(V(0), vel_);
    values_.insert(B(0), bias_);

    // Key timestamps
    stamps_[X(0)] = ts;
    stamps_[V(0)] = ts;
    stamps_[B(0)] = ts;

    // Reset IMU preintegration
    imu_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ = 1;
    init_ = true;
}


void IceNav::update(double ts){
    ROS_INFO_STREAM(correction_count_);

    // Add IMU integration factor
    auto imu_factor = imu_.finishIntegration(ts, correction_count_); // TODO: Make this take two keys instead as arguments
    graph_.add(imu_factor);
    
    auto altitude_factor = AltitudeFactor(X(correction_count_), 0, noiseModel::Isotropic::Sigma(1, 2));
    graph_.add(altitude_factor);

    // Predict pose
    NavState pred_state = imu_.predict(pose_, vel_, bias_);
    pose_ = pred_state.pose();
    vel_ = pred_state.velocity();



    // Add to values_
    values_.insert(X(correction_count_), pose_);
    values_.insert(V(correction_count_), vel_);
    values_.insert(B(correction_count_), bias_);

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
    ts_ = ts;
    pose_ = smoother_.calculateEstimate<Pose3>(X(correction_count_));
    vel_ = smoother_.calculateEstimate<Point3>(V(correction_count_));
    bias_ = smoother_.calculateEstimate<imuBias::ConstantBias>(B(correction_count_));

    writeToFile();

    // Reset IMU preintegration
    imu_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ ++;
}


void IceNav::writeToFile(){
    f_out_ << ts_ << ",";
    f_out_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_out_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_out_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_out_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_out_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_out_ << std::endl;
}


Pose3 IceNav::getLastPose(double& t_pose){
    t_pose = ts_;
    return pose_;
}