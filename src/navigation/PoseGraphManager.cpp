#include "PoseGraphManager.h"
#include "CloudManager.h"

PoseGraphManager::PoseGraphManager(ros::NodeHandle& nh)
    :   imu_integration_(nh), 
        gnss_correction_(nh),
        surface_estimation_(nh),
        smoother_(getParamOrThrow<double>(nh, "/navigation/fixed_lag")),
        lidar_odometry_(nh, smoother_){

    // General config
    getParamOrThrow(nh, "/navigation/initial_acc_bias_sigma", initial_acc_bias_sigma_);
    getParamOrThrow(nh, "/navigation/initial_gyro_bias_sigma", initial_gyro_bias_sigma_);
    
    getParamOrThrow(nh, "/navigation/lever_norm_threshold", lever_norm_threshold_);
    getParamOrThrow(nh, "/navigation/lever_norm_sigma", lever_norm_sigma_);
    getParamOrThrow(nh, "/navigation/lever_altitude_sigma", lever_altitude_sigma_);

    // Output
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);

    // Outstream TODO: Switch to publisher logic?
    fs::path outpath = getParamOrThrow<std::string>(nh, "/outpath");
    std::string nav_path = outpath / "navigation" / "ins.csv";
    makePath(nav_path);

    f_out_ = std::ofstream(nav_path);
    f_out_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz";
    f_out_ << std::endl << std::fixed; 
}


void PoseGraphManager::setCloudManager(CloudManager& cloud_manager){
    cloud_manager_ = &cloud_manager;
    surface_estimation_.setCloudManager(cloud_manager);
    lidar_odometry_.setCloudManager(cloud_manager);
}


int PoseGraphManager::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Integrate new measurement
    imu_integration_.newMeasurement(msg);

    // Check for potential timeout (if we are initialized)
    if (init_ && imu_integration_.timeOut()){
        ROS_WARN("PoseGraphManager: IMU integration timeout, add new state variable");
        return addState(msg->header.stamp.toSec());
    }

    return -1;
}

int PoseGraphManager::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    gnss_correction_.newMeasurement(msg);

    if (init_ && gnss_correction_.isFix()){
        auto gnss_factor = gnss_correction_.getCorrectionFactor(X(state_count_));
        graph_.add(gnss_factor);

        return addState(ts);
    }
    else if (gnss_correction_.isInit() && surface_estimation_.estimateSurface(ts)){
        initialize(ts);
    }

    return -1;
}

void PoseGraphManager::initializeStates(double ts){
    // Pose
    Point2 xy = gnss_correction_.getPosition();
    double z = -surface_estimation_.getSurfaceDistance();
    Point3 pos = Point3(xy.x(), xy.y(), z);

    Rot3 rot = imu_integration_.estimateAttitude();

    pose_ = Pose3(rot, pos);

    // Velcocity
    Point2 v_xy = gnss_correction_.getVelocity();
    vel_ = Point3(v_xy.x(), v_xy.y(), 0);

    // Bias
    Point3 acc_bias = Point3(0, 0, 0); //Point3(-0.03, 0.11, -0.14);
    Point3 gyro_bias = Point3(0, 0, 0); //Point3(-0.002, 0.002, -0.0024);
    bias_ = imuBias::ConstantBias(acc_bias, gyro_bias);

    // Lever arm
    lever_arm_ = pose_.rotation().inverse().rotate(Point3(0, 0, -z));

    // Timestamp
    ts_ = ts;
}

void PoseGraphManager::addPriors(){
    // From GNSS
    auto gnss_factor = gnss_correction_.getCorrectionFactor(X(0));
    graph_.add(gnss_factor); // Planar position prior

    // Bias prior
    auto initial_bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector::Constant(3, initial_acc_bias_sigma_), Vector::Constant(3, initial_gyro_bias_sigma_)).finished()
    );
    graph_.addPrior(B(0), bias_, initial_bias_noise);

    // Lever arm priors
    auto levered_factor = LeveredAltitudeFactor(X(0), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    graph_.add(lever_norm_factor);
}

void PoseGraphManager::initialize(double ts){
    ROS_INFO_STREAM("Initializing navigation system at " << std::fixed << ts);
    
    initializeStates(ts);
    addPriors();

    values_.insert(X(0), pose_);
    values_.insert(V(0), vel_);
    values_.insert(B(0), bias_);
    values_.insert(L(0), lever_arm_); 

    stamps_[X(0)] = ts;
    stamps_[V(0)] = ts;
    stamps_[B(0)] = ts;

    imu_integration_.resetIntegration(ts_, bias_);
    state_count_ = 1;
    init_ = true;
}


/*
Add new factors to graph_ at ts.
*/
void PoseGraphManager::addFactors(double ts){
    // Motion constraint
    auto levered_factor = LeveredAltitudeFactor(X(state_count_), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    // Imu integration
    imu_integration_.finishIntegration(ts);
    auto imu_factor = imu_integration_.getIntegrationFactor(state_count_);
    graph_.add(imu_factor);

    // Try to get a surface estimate
    if (surface_estimation_.estimateSurface(ts_)){
        graph_.add(surface_estimation_.getAltitudeFactor(X(state_count_-1)));
        graph_.add(surface_estimation_.getAttitudeFactor(X(state_count_-1)));
    }
}

void PoseGraphManager::predictStates(double ts){
    // Predict next state
    NavState pred_state = imu_integration_.predict(pose_, vel_, bias_);
    pose_ = pred_state.pose();
    vel_ = pred_state.velocity();
    ts_ = ts;
}

void PoseGraphManager::updateSmoother(double ts){
    // Add to predictions to values_
    values_.insert(X(state_count_), pose_);
    values_.insert(V(state_count_), vel_);
    values_.insert(B(state_count_), bias_);

    // Set timestamps
    stamps_[X(state_count_)] = ts;
    stamps_[V(state_count_)] = ts;
    stamps_[B(state_count_)] = ts;

    // Update smoother
    smoother_.update(graph_, values_, stamps_);
    stamps_.clear();
    values_.clear();
    graph_.resize(0);

    // Update current state
    pose_ = smoother_.calculateEstimate<Pose3>(X(state_count_));
    vel_ = smoother_.calculateEstimate<Point3>(V(state_count_));
    bias_ = smoother_.calculateEstimate<imuBias::ConstantBias>(B(state_count_));
    lever_arm_ = smoother_.calculateEstimate<Point3>(L(0));
}


/*
This is a generic function for adding new state nodes, which typically
happens on GNSS corrections or when the IMU integration times out.  
*/
int PoseGraphManager::addState(double ts){
    ROS_INFO_STREAM("Add state: " << state_count_);

    // Add common factors
    addFactors(ts);

    // Predict next state
    predictStates(ts);

    // Update fixed-lag smoother with new info
    updateSmoother(ts);

    // Generic logic when update is finished
    imu_integration_.resetIntegration(ts_, bias_);

    // Signal new pose
    cloud_manager_->newState(state_count_);

    // Estimate odometry
    auto odometry_factor = lidar_odometry_.estimateOdometry(state_count_);
    if (odometry_factor)
        graph_.add(odometry_factor);

    // Distribute pose somehow
    writeToFile();
    publishPose();

    return state_count_++;
}


std::tuple<double, Pose3> PoseGraphManager::getStampedPose(int idx){
    Key key = X(idx);

    double ts = smoother_.timestamps().at(key);
    Pose3 pose = smoother_.calculateEstimate<Pose3>(key);

    return {ts, pose};
}


void PoseGraphManager::publishPose(){
    if (pose_pub_.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time(ts_);
        msg.pose = poseGtsamToRos(pose_);

        pose_pub_.publish(msg);
    }
}

void PoseGraphManager::writeToFile(){
    f_out_ << ts_ << ",";
    f_out_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_out_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_out_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_out_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_out_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_out_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_out_ << std::endl;
}