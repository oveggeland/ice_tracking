#include "icetrack/navigation/PoseEstimator.h"

PoseEstimator::PoseEstimator(ros::NodeHandle nh, const SensorSystem& sensors)
    : imu_integration_(nh, sensors.imu()), gnss_correction_(nh, sensors.gnss()), surface_estimation_(nh, sensors){

    // Fixed lag smoother
    double fixed_lag = getParamOrThrow<double>(nh, "/navigation/fixed_lag");
    smoother_ = BatchFixedLagSmoother(fixed_lag);

    // Config
    getParamOrThrow(nh, "/navigation/initial_bias_sigma", initial_bias_sigma_);
    getParamOrThrow(nh, "/navigation/initial_velocity_sigma", initial_velocity_sigma_);
    getParamOrThrow(nh, "/navigation/initial_position_sigma", initial_position_sigma_);
    
    getParamOrThrow(nh, "/navigation/lever_norm_threshold", lever_norm_threshold_);
    getParamOrThrow(nh, "/navigation/lever_norm_sigma", lever_norm_sigma_);
    getParamOrThrow(nh, "/navigation/lever_altitude_sigma", lever_altitude_sigma_);

    // Outstream
    fs::path outpath = getParamOrThrow<std::string>(nh, "/outpath");
    std::string nav_path = outpath / "navigation" / "ins.csv";
    makePath(nav_path);

    f_nav_ = std::ofstream(nav_path);
    f_nav_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz";
    f_nav_ << std::endl << std::fixed; 
}

bool PoseEstimator::imuUpdate(){
    if (init_){
        imu_integration_.integrate();
        if (imu_integration_.timeOut()){
            ROS_WARN("PoseEstimator: IMU timeout");

            newCorrection(imu_integration_.getHead());
            return true;
        }
    }
    else
        imu_integration_.initialize();
    
    return false; 
}

bool PoseEstimator::gnssUpdate(){
    if (init_){
        bool fix = gnss_correction_.update();

        if (fix){
            auto gnss_factor = gnss_correction_.getCorrectionFactor(X(correction_count_));
            graph_.add(gnss_factor);

            newCorrection(gnss_correction_.getHead());
            return true;
        }
    }
    else if (gnss_correction_.initialize()){
        double ts = gnss_correction_.getHead();
        if (imu_integration_.isInit() && surface_estimation_.estimateSurface(ts)){
            initialize(ts);
        }
    }

    return false;
}

bool PoseEstimator::lidarUpdate(){
    return false; // Nothing to do here
};

void PoseEstimator::initialize(double ts){
    // Initial state
    ts_ = ts;

    double prior_z = -surface_estimation_.getSurfaceDistance();
    Point3 prior_pos = (Vector3() << gnss_correction_.getPosition(), prior_z).finished();
    pose_ = Pose3(imu_integration_.estimateAttitude(), prior_pos);

    vel_ = (Vector3() << gnss_correction_.getVelocity(), 0).finished();
    bias_ = imuBias::ConstantBias(); // imuBias::ConstantBias(Vector3(-0.03, 0.07, -0.14), Vector3(-0.002, 0.002, -0.0024)); // In case I want to cheat

    lever_arm_ = pose_.rotation().inverse().rotate((Point3() << 0, 0, -prior_z).finished());

    writeToFile(); // Write initial values to file

    // Add prior factors
    graph_.addPrior(B(0), bias_, noiseModel::Isotropic::Sigma(6, initial_bias_sigma_)); // IMU Bias
    graph_.addPrior(V(0), vel_, noiseModel::Isotropic::Sigma(3, initial_velocity_sigma_)); // Velocity from GNSS
    graph_.add(gnss_correction_.getCorrectionFactor(X(0))); // Positiom
    graph_.add(imu_integration_.getAttitudeFactor(X(0))); // Attitude

    // Lever arm priors
    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    graph_.add(lever_norm_factor);

    auto levered_factor = LeveredAltitudeFactor(X(0), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    // Add to values
    values_.insert(X(0), pose_);
    values_.insert(V(0), vel_);
    values_.insert(B(0), bias_);
    values_.insert(L(0), lever_arm_);

    // Key timestamps
    stamps_[X(0)] = ts;
    stamps_[V(0)] = ts;
    stamps_[B(0)] = ts;

    // Reset IMU preintegration
    imu_integration_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ = 1;
    init_ = true;
}

/*
This is a generic uppdate function, called when a new state variable is added. 
*/
void PoseEstimator::newCorrection(double ts){
    ROS_INFO_STREAM(correction_count_);

    // Let us check if LiDAR measurement is available for the previous time step!
    if (surface_estimation_.estimateSurface(ts_)){
        graph_.add(surface_estimation_.getAltitudeFactor(X(correction_count_-1)));
        graph_.add(surface_estimation_.getAttitudeFactor(X(correction_count_-1)));
    }

    // Add altitude constraint factor
    auto levered_factor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    // Finish IMU integration factor
    auto imu_factor = imu_integration_.getIntegrationFactor(ts, correction_count_);
    graph_.add(imu_factor);

    // Predict pose
    NavState pred_state = imu_integration_.predict(pose_, vel_, bias_);
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
    lever_arm_ = smoother_.calculateEstimate<Point3>(L(0));

    // Reset IMU preintegration
    imu_integration_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ ++;
    writeToFile();
}



bool PoseEstimator::isInit(){
    return init_;
}

void PoseEstimator::getCurrentPose(double& ts, Pose3& pose) const{
    pose = imu_integration_.predict(pose_, vel_, bias_).pose();
    ts = imu_integration_.getHead();
}


void PoseEstimator::writeToFile(){
    f_nav_ << ts_ << ",";
    f_nav_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_nav_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_nav_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_nav_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_nav_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_nav_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_nav_ << std::endl;
}