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
    f_out_ << "ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz";
    f_out_ << std::endl << std::fixed; 

    // Sensor handles
    imu_ = ImuHandle();
    gnss_ = GnssHandle();
    lidar_ = LidarHandle();
}


void IceNav::checkCallbackBuffer(){
    for (auto it = callback_buffer_.begin(); it != callback_buffer_.end();){
        double t_msg = it->first;

        if (t_msg > t_safe_)
            break;
        
        it->second(); // Callback 

        it = callback_buffer_.erase(it);
        t_head_ = t_msg; // Maybe this should be done in callback?
    }
}


void IceNav::imuMeasurement(const sensor_msgs::Imu::ConstPtr& msg){
    if (init_){
        imu_.integrate(msg);
    }
    else{
        imu_.init(msg);
    }
}


void IceNav::gnssMeasurement(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (init_){
        auto gnss_factor = gnss_.getCorrectionFactor(msg, X(correction_count_));
        graph_.add(gnss_factor);

        update(msg->header.stamp.toSec());
    }
    else{
        gnss_.init(msg);

        // The system is initialized here when sensors are ready
        if (gnss_.isInit() && lidar_.isInit() && imu_.isInit()){
            initialize(msg->header.stamp.toSec());
        }
    } 
}


void IceNav::pclMeasurement(const sensor_msgs::PointCloud2::ConstPtr& msg){
    double ts = msg->header.stamp.toSec();
    lidar_.addFrame(msg);

    if (!init_)
        lidar_.init(ts);
    else if(request_lidar_correction_ && lidar_.generatePlane(ts_)){
        // Sucessful plane fit, add altitude and attitude factors
        graph_.add(lidar_.getAltitudeFactor(X(correction_count_-1)));
        graph_.add(lidar_.getAttitudeFactor(X(correction_count_-1)));
        
        request_lidar_correction_ = false;
    }    
}


void IceNav::addCallback(double ts, std::function<void()> cb){
    // Is message even valid?
    if (ts < t_head_){
        ROS_WARN("Message older than filter head!");
        return; // Discard measurement
    }

    // Add to buffer
    callback_buffer_[ts] = cb;

    // Check if safe time should be updated
    if (ts - safe_delay_ > t_safe_){
        t_safe_ = ts - safe_delay_;
        checkCallbackBuffer();
    }
}

void IceNav::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceNav::imuMeasurement, this, msg));
}

void IceNav::gnssCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceNav::gnssMeasurement, this, msg));
}

void IceNav::pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    addCallback(msg->header.stamp.toSec(), std::bind(&IceNav::pclMeasurement, this, msg));
}


void IceNav::initialize(double ts){
    // Initial state
    ts_ = ts;

    Point3 prior_pos = (Vector3() << gnss_.getPosition(), lidar_.getAltitude()).finished();
    pose_ = Pose3(imu_.getPriorRot(), prior_pos);

    vel_ = (Vector3() << gnss_.getVelocity(), 0).finished();
    bias_ = imuBias::ConstantBias(); // imuBias::ConstantBias(Vector3(-0.03, 0.07, -0.14), Vector3(-0.002, 0.002, -0.0024)); // In case I want to cheat

    lever_arm_ = pose_.rotation().inverse().rotate((Point3() << 0, 0, -lidar_.getAltitude()).finished());

    writeToFile(); // Write initial values to file

    // Add prior factors
    graph_.addPrior(B(0), bias_, noiseModel::Isotropic::Sigma(6, 0.1)); // Maybe we need this for convergence in the beginning (not sure though)
    graph_.addPrior(V(0), vel_, noiseModel::Isotropic::Sigma(3, 1)); // I don't think we need this
    graph_.add(GPSFactor(X(0), prior_pos, noiseModel::Isotropic::Sigma(3, 2))); // GPS factor for position prior

    graph_.add(imu_.getAttitudeFactor(X(0))); // IMU is more robust here

    // Lever arm priors
    auto lever_norm_factor = Point3NormConstraintFactor(L(0), 25, noiseModel::Isotropic::Sigma(1, 0.1));
    graph_.add(lever_norm_factor);

    auto lever_angle_norm_factor = AngularConstraintFactor(L(0), imu_.getNz(), DEG2RAD(90), noiseModel::Isotropic::Sigma(1, 0.01));
    graph_.add(lever_angle_norm_factor);


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


void IceNav::update(double ts){
    ROS_INFO_STREAM(correction_count_);
    // Add altitude constraint factor
    auto levered_factor = LeveredAltitudeFactor(X(correction_count_), L(0), noiseModel::Isotropic::Sigma(1, 1));
    graph_.add(levered_factor);

    // Flag that LiDAR can add correction when ready
    request_lidar_correction_ = true;

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
    f_out_ << "," << lever_arm_.x() << "," << lever_arm_.y() << "," << lever_arm_.z();
    f_out_ << std::endl;
}