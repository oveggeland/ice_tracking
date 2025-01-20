#include "icetrack/navigation.h"

// Constructors
IceNav::IceNav(){}

IceNav::IceNav(ros::NodeHandle nh, std::shared_ptr<SensorSystem> sensors): nh_(nh), sensors_(sensors){

    // lidar_ = make_shared<Lidar>(sensors_.lidar());
    surface_estimator_ = SurfaceEstimator(nh, sensors);

    // Sensor handles
    imu_ = Imu(nh);
    gnss_ = Gnss(nh);

    // Config
    getParamOrThrow(nh_, "/nav/fixed_lag", fixed_lag_);

    getParamOrThrow(nh_, "/nav/initial_bias_sigma", initial_bias_sigma_);
    getParamOrThrow(nh_, "/nav/initial_velocity_sigma", initial_velocity_sigma_);
    getParamOrThrow(nh_, "/nav/initial_position_sigma", initial_position_sigma_);
    
    getParamOrThrow(nh_, "/nav/lever_norm_threshold", lever_norm_threshold_);
    getParamOrThrow(nh_, "/nav/lever_angle_threshold", lever_angle_threshold_);
    getParamOrThrow(nh_, "/nav/lever_norm_sigma", lever_norm_sigma_);
    getParamOrThrow(nh_, "/nav/lever_angle_sigma", lever_angle_sigma_);
    getParamOrThrow(nh_, "/nav/lever_altitude_sigma", lever_altitude_sigma_);

    getParamOrThrow(nh_, "/nav/gnss/innovation_norm_limit", gnss_innovation_norm_limit_);

    
    getParamOrThrow(nh_, "/nav/ship/use_ship_navigation", use_ship_nav_);


    // Initialize instances 
    graph_ = NonlinearFactorGraph();
    values_ = Values();

    smoother_ = BatchFixedLagSmoother(fixed_lag_);

    // Outstream
    std::string out_path = getParamOrThrow<std::string>(nh_, "/outpath");
    std::string nav_path = joinPath(out_path, "nav/nav.csv");
    makePath(nav_path);

    f_nav_ = std::ofstream(nav_path);
    f_nav_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz";
    f_nav_ << std::endl << std::fixed; 
}

void IceNav::imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg){
    if (init_){
        imu_.integrate(msg);
        if (imu_.timeOut()){
            ROS_WARN_STREAM(std::fixed << msg->header.stamp.toSec() << ": IMU timeout");
            update(msg->header.stamp.toSec());
        }
    }
    else{
        imu_.init(msg);
    }
}

void IceNav::gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();

    if (init_){
        Point2 xy = gnss_.getMeasurement(msg);
        Point2 innovation = xy - pose_.translation().head<2>();

        if (innovation.norm() < gnss_innovation_norm_limit_){
            auto gnss_factor = gnss_.getCorrectionFactor(xy, X(correction_count_));
            graph_.add(gnss_factor);

            update(ts);
        }
    }
    else{
        gnss_.init(msg);

        // The system is initialized here when sensors are ready (position and velocity from gnss, orientation from IMU and altitude from surface estimator)
        if (gnss_.isInit() && imu_.isInit() && surface_estimator_.estimateSurface(ts))
            initialize(ts);
    } 
}

/**
 * This function can be used to provide extra information to the navigation system from the ship. 
 * Consider this cheating for the stand-alone concept, but for data generation this is gold. 
 */
void IceNav::shipNavigationMeasurement(const icetrack::ShipNavigation::ConstPtr& msg){
    ROS_INFO("SHIP NAC");
    if (!use_ship_nav_)
        return;
}


bool IceNav::isInit(){
    return init_;
}

Pose3 IceNav::getPose(){
    return pose_;
}

void IceNav::initialize(double ts){
    // Initial state
    ts_ = ts;

    double prior_z = -surface_estimator_.getSurfaceDistance();
    Point3 prior_pos = (Vector3() << gnss_.getPosition(), prior_z).finished();
    pose_ = Pose3(imu_.getPriorRot(), prior_pos);

    vel_ = (Vector3() << gnss_.getVelocity(), 0).finished();
    bias_ = imuBias::ConstantBias(); // imuBias::ConstantBias(Vector3(-0.03, 0.07, -0.14), Vector3(-0.002, 0.002, -0.0024)); // In case I want to cheat

    lever_arm_ = pose_.rotation().inverse().rotate((Point3() << 0, 0, -prior_z).finished());

    writeToFile(); // Write initial values to file

    // Add prior factors
    graph_.addPrior(B(0), bias_, noiseModel::Isotropic::Sigma(6, initial_bias_sigma_)); // Maybe we need this for convergence in the beginning (not sure though)
    graph_.addPrior(V(0), vel_, noiseModel::Isotropic::Sigma(3, initial_velocity_sigma_)); // I don't think we need this
    graph_.add(GPSFactor(X(0), prior_pos, noiseModel::Isotropic::Sigma(3, initial_position_sigma_))); // GPS factor for position prior

    graph_.add(imu_.getAttitudeFactor(X(0))); // IMU is more robust here

    // Lever arm priors
    auto lever_norm_factor = NormConstraintFactor(L(0), lever_norm_threshold_, noiseModel::Isotropic::Sigma(1, lever_norm_sigma_));
    graph_.add(lever_norm_factor);

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
    imu_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ = 1;
    init_ = true;
}

/*
This is a generic uppdate function, called when a new state variable is added. 
*/
void IceNav::update(double ts){
    ROS_INFO_STREAM(correction_count_);

    // Let us check if LiDAR measurement is available for the previous time step!
    if (surface_estimator_.estimateSurface(ts_)){
        graph_.add(surface_estimator_.getAltitudeFactor(X(correction_count_-1)));
        graph_.add(surface_estimator_.getAttitudeFactor(X(correction_count_-1)));
    }

    // Add altitude constraint factor
    auto levered_factor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, lever_altitude_sigma_));
    graph_.add(levered_factor);

    // Finish IMU integration factor
    auto imu_factor = imu_.finishIntegration(ts, correction_count_); // TODO: Make this take two keys instead as arguments
    graph_.add(imu_factor);

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
    lever_arm_ = smoother_.calculateEstimate<Point3>(L(0));

    // Reset IMU preintegration
    imu_.resetIntegration(ts, bias_);

    // Control parameters
    correction_count_ ++;
    writeToFile();
}


void IceNav::writeToFile(){
    f_nav_ << ts_ << ",";
    f_nav_ << pose_.translation()[0] << "," << pose_.translation()[1] << "," << pose_.translation()[2] << ",";
    f_nav_ << vel_[0] << "," << vel_[1] << "," << vel_[2] << ",";
    f_nav_ << pose_.rotation().ypr()[2] << "," << pose_.rotation().ypr()[1] << "," << pose_.rotation().ypr()[0] << ",";
    f_nav_ << bias_.accelerometer()[0] << "," << bias_.accelerometer()[1] << "," << bias_.accelerometer()[2] << ",";
    f_nav_ << bias_.gyroscope()[0] << "," << bias_.gyroscope()[1] << "," << bias_.gyroscope()[2];
    f_nav_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_nav_ << std::endl;
}