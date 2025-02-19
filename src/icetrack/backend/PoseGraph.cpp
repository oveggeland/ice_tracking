#include "PoseGraph.h"

PoseGraph::PoseGraph(ros::NodeHandle& nh)
    : imu_integration_(nh), gnss_correction_(nh), 
    surface_correction_(nh), pose_graph_output_(nh){

    // Initialize smoother
    double lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = BatchFixedLagSmoother(lag); // TODO: Isam2?

    readParams(nh);
}   

void PoseGraph::readParams(const ros::NodeHandle& nh){
    getParamOrThrow(nh, "/navigation/initial_acc_bias_sigma", initial_acc_bias_sigma_);
    getParamOrThrow(nh, "/navigation/initial_gyro_bias_sigma", initial_gyro_bias_sigma_);
    
    getParamOrThrow(nh, "/navigation/lever_norm_threshold", lever_norm_threshold_);
    getParamOrThrow(nh, "/navigation/lever_norm_sigma", lever_norm_sigma_);
    getParamOrThrow(nh, "/navigation/lever_altitude_sigma", lever_altitude_sigma_);
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

    auto odom_factor = BetweenFactor<Pose3>(X(idx0), X(idx1), Pose3(T_align), noise_model);
    factors_.add(odom_factor);
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
    
    // Output pose
    pose_graph_output_.outputState(state_);
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
}

void PoseGraph::updateValues(int idx){
    // Add to predictions to values_
    values_.insert(X(idx), pose_);
    values_.insert(V(idx), vel_);
    values_.insert(B(idx), bias_);

    if (!init_)
        values_.insert(L(0), lever_arm_);
}

void PoseGraph::updateTimeStamps(int idx, double ts){
    stamps_[X(idx)] = ts;
    stamps_[V(idx)] = ts;
    stamps_[B(idx)] = ts;
}

void PoseGraph::updateFactors(int idx, double ts){
    // Motion constraint
    auto levered_factor = LeveredAltitudeFactor(X(idx), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    factors_.add(levered_factor);

    // Imu integration
    auto imu_factor = imu_integration_.getIntegrationFactor(idx);
    factors_.add(imu_factor);
}

void PoseGraph::updateSmoother(){
    smoother_.update(factors_, values_, stamps_);
    stamps_.clear();
    values_.clear();
    factors_.resize(0);
}