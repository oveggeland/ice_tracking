#include "PoseGraph.h"

PoseGraph::PoseGraph(ros::NodeHandle& nh)
    : imu_integration_(nh), gnss_correction_(nh), 
    surface_correction_(nh), pose_graph_output_(nh){

    // Initialize smoother
    double lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = IncrementalFixedLagSmoother(lag);

    readParams(nh);
}   

void PoseGraph::readParams(const ros::NodeHandle& nh){
    getParamOrThrow(nh, "/navigation/initial_acc_bias_sigma", initial_acc_bias_sigma_);
    getParamOrThrow(nh, "/navigation/initial_gyro_bias_sigma", initial_gyro_bias_sigma_);
    
    getParamOrThrow(nh, "/navigation/lever_norm_threshold", lever_norm_threshold_);
    getParamOrThrow(nh, "/navigation/lever_norm_sigma", lever_norm_sigma_);
    getParamOrThrow(nh, "/navigation/lever_altitude_sigma", lever_altitude_sigma_);

    getParamOrThrow(nh, "/navigation/hot_start_delay", hot_start_delay_);
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
        auto gnss_factor = gnss_correction_.getCorrectionFactor(X(state_.idx+1));
        factors_.add(gnss_factor);

        addState(ts);
    }
    else{
        // Reset IMU and wait for planeFitCallback to finish initialization
        imu_integration_.resetIntegration(ts, state_.bias);
        state_.ts = ts;
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
    }
    else
        initialize();
}

void PoseGraph::initialize(){
    ROS_INFO_STREAM("Initialize pose graph at " << std::fixed << state_.ts);
    initializeState();

    updateValues(0);
    updateTimeStamps(0, state_.ts);
    addPriors(0);

    state_.idx = 0;
    init_ = true;
}


void PoseGraph::addState(double ts){
    state_.idx++;
    std::cout << "Add state " << state_.idx << std::endl;

    // Finish integration and predict state
    imu_integration_.finishIntegration(ts);
    imu_integration_.predictState(state_);

    // Update a whole bunch of shit
    updateTimeStamps(state_.idx, ts);
    updateValues(state_.idx);
    updateFactors(state_.idx, ts);

    // Update pose graph and current state
    if (state_.idx > hot_start_delay_){
        updateSmoother();
        updateState(state_.idx);
    }

    // Reset integration
    imu_integration_.resetIntegration(ts, state_.bias);
    state_.ts = ts;
    
    // Output pose
    pose_graph_output_.outputState(state_);
}

void PoseGraph::addPriors(int idx){
    // GNSS prior
    auto gnss_factor = gnss_correction_.getCorrectionFactor(X(idx));
    factors_.add(gnss_factor);

    // Altitude prior
    factors_.add(surface_correction_.getAltitudeFactor(X(idx)));

    // Attitude prior
    factors_.add(imu_integration_.getAttitudeFactor(X(idx)));

    // Bias prior
    auto initial_bias_noise = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector::Constant(3, initial_acc_bias_sigma_), Vector::Constant(3, initial_gyro_bias_sigma_)).finished()
    );
    factors_.addPrior(B(idx), state_.bias, initial_bias_noise);

    // Lever arm priors
    auto levered_factor = LeveredAltitudeFactor(X(idx), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    factors_.add(levered_factor);

    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    factors_.add(lever_norm_factor);
}


void PoseGraph::initializeState(){
    Point2 xy = gnss_correction_.getPosition();
    double z = -surface_correction_.getSurfaceDistance();

    state_.pose = Pose3(
        imu_integration_.estimateAttitude(),
        Point3(xy.x(), xy.y(), z)
    );
    
    state_.velocity = Vector3::Zero();
    state_.bias = imuBias::ConstantBias(
        Vector3(-0.035210,0.085232,-0.138853),
        Vector3(-0.002100,0.002172,-0.002661)
    );

    state_.lever_arm = state_.pose.rotation().inverse().rotate(Point3(0, 0, -z));
}

void PoseGraph::updateState(int idx){
    state_.pose = smoother_.calculateEstimate<Pose3>(X(idx));
    state_.velocity = smoother_.calculateEstimate<Point3>(V(idx));
    state_.bias = smoother_.calculateEstimate<imuBias::ConstantBias>(B(idx));
    state_.lever_arm = smoother_.calculateEstimate<Point3>(L(0));
}

void PoseGraph::updateValues(int idx){
    // Add to predictions to values_
    values_.insert(X(idx), state_.pose);
    values_.insert(V(idx), state_.velocity);
    values_.insert(B(idx), state_.bias);

    if (!init_)
        values_.insert(L(0), state_.lever_arm);
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