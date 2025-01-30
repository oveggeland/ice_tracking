#include "PoseEstimator.h"

PoseEstimator::PoseEstimator(ros::NodeHandle nh)
    :   imu_integration_(nh), 
        gnss_correction_(nh), 
        point_buffer_(nh), 
        frame_buffer_(nh, point_buffer_),
        surface_estimation_(nh, point_buffer_), 
        lidar_odometry_(nh, frame_buffer_){

    // Fixed lag smoother
    double lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = BatchFixedLagSmoother(lag);

    // Setup sequencer and subscribers
    sequencer_ = CallbackSequencer(getParamOrThrow<double>(nh, "/navigation/safe_delay"));

    std::string imu_topic = getParamOrThrow<std::string>(nh, "/imu_topic");
    std::string gnss_topic = getParamOrThrow<std::string>(nh, "/gnss_topic");
    std::string lidar_topic = getParamOrThrow<std::string>(nh, "/lidar_topic");

    imu_sub_ = nh.subscribe(imu_topic, 2000, &PoseEstimator::imuCallback, this);
    gnss_sub_ = nh.subscribe(gnss_topic, 10, &PoseEstimator::gnssCallback, this);
    lidar_sub_ = nh.subscribe(lidar_topic, 100, &PoseEstimator::lidarCallback, this);

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


// Subscriber callbacks. All we do is add the callbacks to the sequencer to assert chronological order of messages
void PoseEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&PoseEstimator::imuSafeCallback, this, msg));
};
void PoseEstimator::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&PoseEstimator::gnssSafeCallback, this, msg));
};
void PoseEstimator::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sequencer_.addCallback(msg->header.stamp.toSec(), std::bind(&PoseEstimator::lidarSafeCallback, this, msg));
};


void PoseEstimator::imuSafeCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Integrate new measurement
    imu_integration_.newMeasurement(msg);

    // Check for potential timeout (if we are initialized)
    if (init_ && imu_integration_.timeOut()){
        ROS_WARN("PoseEstimator: IMU integration timeout, add new state variable");
        addState(msg->header.stamp.toSec());
    }
}


void PoseEstimator::gnssSafeCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    gnss_correction_.newMeasurement(msg);

    if (init_ && gnss_correction_.isFix()){
        auto gnss_factor = gnss_correction_.getCorrectionFactor(X(state_count_));
        graph_.add(gnss_factor);

        addState(ts);
    }
    else if (gnss_correction_.isInit() && surface_estimation_.estimateSurface(ts)){
        initialize(ts);
    }
}


void PoseEstimator::lidarSafeCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    point_buffer_.addLidarPoints(msg);
};


void PoseEstimator::initializeState(){
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
    Point3 acc_bias = Point3(0, 0, 0);      // Point3(-0.03, 0.07, -0.14)
    Point3 gyro_bias = Point3(0, 0, 0);     // Point3(-0.002, 0.002, -0.0024)
    bias_ = imuBias::ConstantBias(acc_bias, gyro_bias);

    // Lever arm
    lever_arm_ = pose_.rotation().inverse().rotate(Point3(0, 0, -z));
}

void PoseEstimator::addPriors(){
    // From GNSS
    auto gnss_factor = gnss_correction_.getCorrectionFactor(X(0));
    graph_.add(gnss_factor); // Planar position prior

    // Bias prior
    auto initial_bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector::Constant(3, initial_acc_bias_sigma_), Vector::Constant(3, initial_gyro_bias_sigma_)).finished()
    );
    graph_.addPrior(B(0), bias_, initial_bias_noise);

    // Lever arm priors
    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    graph_.add(lever_norm_factor);
}

void PoseEstimator::initialize(double ts){
    ROS_INFO_STREAM("Initializing navigation system at " << std::fixed << ts);
    
    initializeState();
    addPriors();
    addState(ts);
    init_ = true;
}

/*
This is a generic uppdate function, called when a new state variable is added. 
*/
void PoseEstimator::addState(double ts){
    ROS_INFO_STREAM("Add state: " << state_count_);

    // Add altitude constraint
    auto levered_factor = LeveredAltitudeFactor(X(state_count_), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    if (state_count_ > 0){
        // Imu integration
        imu_integration_.finishIntegration(ts);
        auto imu_factor = imu_integration_.getIntegrationFactor(state_count_);
        graph_.add(imu_factor);

        // Always add surface correction for previous state (symmetry reasons)
        if (surface_estimation_.estimateSurface(ts_)){
            graph_.add(surface_estimation_.getAltitudeFactor(X(state_count_-1)));
            graph_.add(surface_estimation_.getAttitudeFactor(X(state_count_-1)));
        }

        // Predict next state
        NavState pred_state = imu_integration_.predict(pose_, vel_, bias_);
        pose_ = pred_state.pose();
        vel_ = pred_state.velocity();
    }
    else{
        // Only on initial state
        values_.insert(L(0), lever_arm_);
    }

    // Add to values_
    values_.insert(X(state_count_), pose_);
    values_.insert(V(state_count_), vel_);
    values_.insert(B(state_count_), bias_);

    // Key timestamps
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

    // New states? Let's generate a new LiDAR frame
    if (state_count_ > 0){
        Pose3 prev_pose = smoother_.calculateEstimate<Pose3>(X(state_count_-1));
        frame_buffer_.createFrame(state_count_, ts_, ts, prev_pose, pose_);
    }

    // Reset IMU preintegration
    imu_integration_.resetIntegration(ts, bias_);

    // Control parameters
    ts_ = ts;
    state_count_ ++;

    writeToFile();
    publishPose();
}


void PoseEstimator::publishPose(){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time(ts_);
    msg.pose = poseGtsamToRos(pose_);

    pose_pub_.publish(msg);
}

void PoseEstimator::writeToFile(){
    f_out_ << ts_ << ",";
    f_out_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_out_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_out_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_out_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_out_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_out_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_out_ << std::endl;
}