#include "PoseGraph.h"

PoseGraph::PoseGraph(ros::NodeHandle& nh)
    :   imu_integration_(nh), gnss_correction_(nh){

    // Initialize smoother
    double lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = BatchFixedLagSmoother(lag);

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
    f_out_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz,Dx,Dy";
    f_out_ << std::endl << std::fixed; 
}


void PoseGraph::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    // Integrate new measurement
    imu_integration_.newMeasurement(msg);

    // Check for potential timeout (if we are initialized)
    if (isInit() && imu_integration_.timeOut()){
        ROS_WARN("PoseGraph: IMU integration timeout, add new state variable");
        addState(msg->header.stamp.toSec());
    }
}

void PoseGraph::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    gnss_correction_.newMeasurement(msg);

    if (!gnss_correction_.isFix())
        return;
    else if (init_){
        auto gnss_factor = gnss_correction_.getCorrectionFactor(X(state_idx_));
        graph_.add(gnss_factor);

        addState(ts);
    }
    else if (prior_z_ != 0.0){
        initialize(ts);
    }
}

void PoseGraph::planeFitCallback(int state_idx, Eigen::Vector4d plane_coeffs){
    prior_z_ = -abs(plane_coeffs[3]);
}

void PoseGraph::initializeState(double ts){
    // Pose
    Point2 xy = gnss_correction_.getPosition();
    double z = -prior_z_;
    Point3 pos = Point3(xy.x(), xy.y(), z);

    Rot3 rot = imu_integration_.estimateAttitude();

    pose_ = Pose3(rot, pos);

    // Velcocity
    vel_ = Point3(0, 0, 0);

    // Bias
    Point3 acc_bias = Point3(0, 0, 0); //Point3(-0.03, 0.11, -0.14);
    Point3 gyro_bias = Point3(0, 0, 0); //Point3(-0.002, 0.002, -0.0024);
    bias_ = imuBias::ConstantBias(acc_bias, gyro_bias);

    // Lever arm
    lever_arm_ = pose_.rotation().inverse().rotate(Point3(0, 0, -z));

    // Ice state
    ice_drift_ = Point2(0, 0);

    // Timestamp
    state_idx_ = 0;
    ts_ = ts;
}

void PoseGraph::addPriors(double ts){
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

void PoseGraph::initialize(double ts){
    ROS_INFO_STREAM("Initializing navigation system at " << std::fixed << ts);

    initializeState(ts);
    addVariables(ts);
    addPriors(ts);

    imu_integration_.resetIntegration(ts_, bias_);
    init_ = true;
}


void PoseGraph::predictState(double ts){
    // Predict next state
    imu_integration_.finishIntegration(ts);
    NavState pred_state = imu_integration_.predict(pose_, vel_, bias_);
    pose_ = pred_state.pose();
    vel_ = pred_state.velocity();
}

void PoseGraph::addVariables(double ts){
    // Add to predictions to values_
    values_.insert(X(state_idx_), pose_);
    values_.insert(V(state_idx_), vel_);
    values_.insert(B(state_idx_), bias_);

    // Set timestamps
    stamps_[X(state_idx_)] = ts;
    stamps_[V(state_idx_)] = ts;
    stamps_[B(state_idx_)] = ts;

    // Only add lever arm on initial s
    if (state_idx_ == 0)
        values_.insert(L(0), lever_arm_);

    // Optionally add ice drift
    if (estimate_ice_drift_){
        values_.insert(D(state_idx_), ice_drift_);
        stamps_[D(state_idx_)] = ts;
    }
}

void PoseGraph::addFactors(double ts){
    // Motion constraint
    auto levered_factor = LeveredAltitudeFactor(X(state_idx_), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    // Imu integration
    auto imu_factor = imu_integration_.getIntegrationFactor(state_idx_);
    graph_.add(imu_factor);

    // Drift estimation
    if (estimate_ice_drift_){
        double dt = ts - ts_;
        auto noise = noiseModel::Isotropic::Sigma(2, 0.05*dt);

        auto cv_factor = BetweenFactor<Point2>(D(state_idx_-1), D(state_idx_), Point2(0, 0), noise);
        graph_.add(cv_factor);
    }
}

void PoseGraph::updateSmoother(double ts){
    // Update smoother
    smoother_.update(graph_, values_, stamps_);
    stamps_.clear();
    values_.clear();
    graph_.resize(0);

    // Update current state
    pose_ = smoother_.calculateEstimate<Pose3>(X(state_idx_));
    vel_ = smoother_.calculateEstimate<Point3>(V(state_idx_));
    bias_ = smoother_.calculateEstimate<imuBias::ConstantBias>(B(state_idx_));
    lever_arm_ = smoother_.calculateEstimate<Point3>(L(0));

    if (estimate_ice_drift_)
        ice_drift_ = smoother_.calculateEstimate<Point2>(D(state_idx_));
}


/*
This is a generic function for adding new state nodes, which typically
happens on GNSS corrections or when the IMU integration times out.  
*/
void PoseGraph::addState(double ts){
    state_idx_ ++;

    // Predict new state
    predictState(ts);

    // variables and factors
    addVariables(ts);
    addFactors(ts);

    // Update fixed-lag smoother with new info
    updateSmoother(ts);

    // Generic after update
    imu_integration_.resetIntegration(ts, bias_);
    ts_ = ts;
    
    // Distribute pose somehow
    writeToFile();
    publishPose();

    ROS_INFO_STREAM("State added: " << state_idx_);
}

void PoseGraph::publishPose(){
    if (pose_pub_.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time(ts_);
        msg.pose = poseGtsamToRos(pose_);

        pose_pub_.publish(msg);
    }
}

void PoseGraph::writeToFile(){
    f_out_ << ts_ << ",";
    f_out_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_out_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_out_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_out_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_out_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_out_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_out_ << "," << ice_drift_.x() << "," << ice_drift_.y();
    f_out_ << std::endl;
}