#include "backend/PoseGraphOutput.h"

PoseGraphOutput::PoseGraphOutput(ros::NodeHandle& nh){
    setupBroadcaster(nh);
    setupPublisher(nh);
    setupFileStream(nh);
}

void PoseGraphOutput::setupBroadcaster(const ros::NodeHandle& nh){
    tf_.header.frame_id = "map";
    tf_.child_frame_id = "body";
}

void PoseGraphOutput::setupPublisher(ros::NodeHandle& nh){
    int pose_queue_size = getParamOrThrow<int>(nh, "/pose_queue_size");
    std::string pose_topic = getParamOrThrow<std::string>(nh, "/pose_topic");
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, pose_queue_size);

    pose_msg_.header.frame_id = "map";
}

void PoseGraphOutput::setupFileStream(const ros::NodeHandle& nh){
    std::string ws = getParamOrThrow<std::string>(nh, "/workspace");
    std::string exp = getParamOrThrow<std::string>(nh, "/exp");
    std::string fpath = joinPaths({ws, exp, "navigation", "state.csv"});
    makePath(fpath);

    f_out_ = std::ofstream(fpath);
    f_out_ << "idx,ts,x,y,z,vx,vy,vz,roll,pitch,yaw,bax,bay,baz,bgx,bgy,bgz,Lx,Ly,Lz";
    f_out_ << std::endl << std::fixed; 
}

void PoseGraphOutput::outputState(const PoseGraphState& state){
    broadcastTransform(state);
    publishPose(state);
    writeToFile(state);
}

void PoseGraphOutput::publishPose(const PoseGraphState& state){
    if (pose_pub_.getNumSubscribers() > 0){
        pose_msg_.header.stamp = ros::Time(state.ts);
        pose_msg_.pose = poseGtsamToRos(state.pose);

        pose_pub_.publish(pose_msg_);
    }
}

void PoseGraphOutput::broadcastTransform(const PoseGraphState& state){
    tf_.header.stamp = ros::Time(state.ts);

    Pose3 pose = state.pose;
    tf_.transform.translation.x = pose.x();
    tf_.transform.translation.y = pose.y();
    tf_.transform.translation.z = pose.z();

    Eigen::Quaterniond q = pose.rotation().toQuaternion();
    tf_.transform.rotation.x = q.x();
    tf_.transform.rotation.y = q.y();
    tf_.transform.rotation.z = q.z();
    tf_.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(tf_);
}

void PoseGraphOutput::writeToFile(const PoseGraphState& state){
    // Index
    f_out_ << state.idx;

    // Timestamp
    f_out_ << "," << state.ts;

    // Position
    Vector3 pos = state.pose.translation();
    f_out_ << "," << pos(0) << "," << pos(1) << "," << pos(2);

    // Velocity
    Vector3 vel = state.velocity;
    f_out_ << "," << vel(0) << "," << vel(1) << "," << vel(2);
    
    // Orientation
    Vector3 rpy = state.pose.rotation().rpy();
    f_out_ << "," << rpy(0) << "," << rpy(1) << "," << rpy(2);

    // Acc bias
    Vector3 b_acc = state.bias.accelerometer();
    f_out_ << "," << b_acc(0) << "," << b_acc(1) << "," << b_acc(2);

    // Gyro bias
    Vector3 b_gyro = state.bias.gyroscope();
    f_out_ << "," << b_gyro(0) << "," << b_gyro(1) << "," << b_gyro(2);

    // Lever arm
    Vector3 lever_arm = state.lever_arm;
    f_out_ << "," << lever_arm(0) << "," << lever_arm(1) << "," << lever_arm(2);
    
    // Line break
    f_out_ << std::endl;
}