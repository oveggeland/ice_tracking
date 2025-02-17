#include "PoseGraph.h"

PoseGraph::PoseGraph(ros::NodeHandle& nh)
    : imu_integration_(nh), gnss_correction_(nh), surface_correction_(nh){

    // Initialize smoother
    double lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = BatchFixedLagSmoother(lag); // TODO: Isam?

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
    if (init_ && imu_integration_.timeOut()){
        ROS_WARN("PoseGraph: IMU integration timeout, add new state variable");
        addState(msg->header.stamp.toSec());
    }
}

void PoseGraph::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    // Parse new gnss measurement
    gnss_correction_.newMeasurement(msg);
    double ts = msg->header.stamp.toSec();

    // Is measurement valid?
    if (!gnss_correction_.isFix())
        return;

    if (init_){
        // Woho! add factor and update
        auto gnss_factor = gnss_correction_.getCorrectionFactor(X(state_idx_+1));
        factors_.add(gnss_factor);

        addState(ts);
    }
    else{
        // Reset IMU and wait for planeFitCallback to finish initialization
        imu_integration_.resetIntegration(ts, bias_);
        ts_ = ts;
    }
}

void PoseGraph::odometryCallback(int idx0, int idx1, Eigen::Matrix4d T_align){
    auto noise_model = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector3::Constant(0.1), Vector3::Constant(2.0)).finished()
    );

    if (estimate_ice_drift_){
        double dt = getTimeStamp(idx1) - getTimeStamp(idx0);
        auto odom_factor = IceOdometryFactor(X(idx0), X(idx1), D(idx0), Pose3(T_align), dt, noise_model);
        factors_.add(odom_factor);
    }
    else{
        auto odom_factor = BetweenFactor<Pose3>(X(idx0), X(idx1), Pose3(T_align), noise_model);
        factors_.add(odom_factor);
    }
}

void PoseGraph::surfaceCallback(int state_idx, const Eigen::Vector4d& plane_coeffs){
    surface_correction_.setPlaneCoeffs(plane_coeffs);

    if (init_){
        factors_.add(surface_correction_.getAltitudeFactor(X(state_idx)));
        factors_.add(surface_correction_.getAttitudeFactor(X(state_idx)));
    }
    else
        initialize();
}

void PoseGraph::initialize(){
    ROS_INFO_STREAM("Initialize pose graph at " << std::fixed << ts_);
    initializeState();

    updateValues(0);
    updateTimeStamps(0, ts_);
    addPriors(0);

    state_idx_ = 0;
    init_ = true;
}


void PoseGraph::addState(double ts){
    state_idx_++;
    ROS_INFO_STREAM("Add state " << state_idx_);

    // Extrapolate IMU integration to 'ts'
    imu_integration_.finishIntegration(ts);

    // Predict-update cycle
    predictState(ts);
    updateTimeStamps(state_idx_, ts);
    updateValues(state_idx_);
    updateFactors(state_idx_, ts);
    updateSmoother();
    updateState(state_idx_);

    // Reset integration
    imu_integration_.resetIntegration(ts, bias_);
    ts_ = ts;
    
    // Distribute pose
    writeToFile();
    publishPose();
}

void PoseGraph::addPriors(int idx){
    // GNSS prior
    auto gnss_factor = gnss_correction_.getCorrectionFactor(X(idx));
    factors_.add(gnss_factor);

    // Altitude prior
    factors_.add(surface_correction_.getAltitudeFactor(X(idx)));
    factors_.add(surface_correction_.getAttitudeFactor(X(idx)));

    // Bias prior
    auto initial_bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector::Constant(3, initial_acc_bias_sigma_), Vector::Constant(3, initial_gyro_bias_sigma_)).finished()
    );
    factors_.addPrior(B(idx), bias_, initial_bias_noise);

    // Lever arm priors
    auto levered_factor = LeveredAltitudeFactor(X(idx), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    factors_.add(levered_factor);

    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    factors_.add(lever_norm_factor);

    // Drift prior
    if (estimate_ice_drift_){
        factors_.addPrior(D(idx), ice_drift_, noiseModel::Isotropic::Sigma(2, 2));
    }
}

void PoseGraph::initializeState(){
    Point2 xy = gnss_correction_.getPosition();
    double z = -surface_correction_.getSurfaceDistance();
    pose_ = Pose3(
        imu_integration_.estimateAttitude(), 
        Point3(xy.x(), xy.y(), z)
    );

    lever_arm_ = pose_.rotation().inverse().rotate(Point3(0, 0, -z));
}

void PoseGraph::predictState(double ts){
    NavState pred_state = imu_integration_.predict(pose_, vel_, bias_);
    pose_ = pred_state.pose();
    vel_ = pred_state.velocity();
}

void PoseGraph::updateState(int idx){
    pose_ = smoother_.calculateEstimate<Pose3>(X(idx));
    vel_ = smoother_.calculateEstimate<Point3>(V(idx));
    bias_ = smoother_.calculateEstimate<imuBias::ConstantBias>(B(idx));
    lever_arm_ = smoother_.calculateEstimate<Point3>(L(0));

    if (estimate_ice_drift_)
        ice_drift_ = smoother_.calculateEstimate<Point2>(D(idx));
}

void PoseGraph::updateValues(int idx){
    // Add to predictions to values_
    values_.insert(X(idx), pose_);
    values_.insert(V(idx), vel_);
    values_.insert(B(idx), bias_);

    if (!init_)
        values_.insert(L(0), lever_arm_);

    if (estimate_ice_drift_)
        values_.insert(D(idx), ice_drift_);
}

void PoseGraph::updateTimeStamps(int idx, double ts){
    stamps_[X(idx)] = ts;
    stamps_[V(idx)] = ts;
    stamps_[B(idx)] = ts;

    if (estimate_ice_drift_)
        stamps_[D(idx)] = ts;
}

void PoseGraph::updateFactors(int idx, double ts){
    // Motion constraint
    auto levered_factor = LeveredAltitudeFactor(X(idx), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    factors_.add(levered_factor);

    // Imu integration
    auto imu_factor = imu_integration_.getIntegrationFactor(idx);
    factors_.add(imu_factor);

    // Drift estimation
    if (estimate_ice_drift_){
        double dt = ts - ts_;
        auto noise = noiseModel::Isotropic::Sigma(2, 0.05*dt);

        auto cv_factor = BetweenFactor<Point2>(D(idx-1), D(idx), Point2(0, 0), noise);
        factors_.add(cv_factor);
    }
}

void PoseGraph::updateSmoother(){
    smoother_.update(factors_, values_, stamps_);
    stamps_.clear();
    values_.clear();
    factors_.resize(0);
}


void PoseGraph::publishPose(){
    if (pose_pub_.getNumSubscribers() > 0){
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time(ts_);
        msg.header.frame_id = "map";
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