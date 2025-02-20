#include "OdometryEstimator.h"


OdometryEstimator::OdometryEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer)
                        : pose_graph_(pose_graph), frame_buffer_(frame_buffer) {
    // Import config
    getParamOrThrow(nh, "/lidar_odometry/enabled", enabled_);
    getParamOrThrow(nh, "/lidar_odometry/frame_interval", frame_interval_);
    getParamOrThrow(nh, "/lidar_odometry/min_frame_size", min_frame_size_);
    getParamOrThrow(nh, "/lidar_odometry/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/lidar_odometry/icp_threshold", icp_threshold_);
    getParamOrThrow(nh, "/lidar_odometry/icp_min_fitness", icp_min_fitness_);
};

void OdometryEstimator::estimateOdometry(int idx1) {
    if (!enabled_)
        return;
        
    int idx0 = idx1 - frame_interval_;

    // Get frames
    auto frame0 = frame_buffer_.getFrame(idx0);
    auto frame1 = frame_buffer_.getFrame(idx1);

    // Check validity and size of frames
    if (!frame0 || !frame1 || frame0->size() < min_frame_size_ || frame1->size() < min_frame_size_)
        return;
    
    // Get pose and initial alignment
    gtsam::Pose3 pose0, pose1;
    if (!pose_graph_.poseQuery(idx0, pose0) || !pose_graph_.poseQuery(idx1, pose1))
        return; // Pose query failed...
    Eigen::Matrix4d T_initial = pose0.between(pose1).matrix();

    // Perform ICP alignment
    auto result = open3d::pipelines::registration::RegistrationICP(
        frame1->local(), frame0->local(), icp_threshold_, T_initial.matrix(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1.0e-6, 1.0e-6, 30)
    );
    Eigen::Matrix4d T_align = result.transformation_;

    // Check if ICP converged
    if (result.fitness_ < icp_min_fitness_)  // Fitness threshold (tune based on environment)
        return;

    // visualizeAlignment(cloud0_ds, cloud1_ds, T_initial, T_align);
    pose_graph_.odometryCallback(idx0, idx1, T_align);
}