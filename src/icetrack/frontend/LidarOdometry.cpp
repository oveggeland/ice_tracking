#include "LidarOdometry.h"


LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer)
                        : pose_graph_(pose_graph), frame_buffer_(frame_buffer) {
    // Import config
    getParamOrThrow(nh, "/lidar_odometry/frame_interval", frame_interval_);
    getParamOrThrow(nh, "/lidar_odometry/min_frame_size", min_frame_size_);
    getParamOrThrow(nh, "/lidar_odometry/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/lidar_odometry/icp_threshold", icp_threshold_);
    getParamOrThrow(nh, "/lidar_odometry/icp_min_fitness", icp_min_fitness_);
};

void LidarOdometry::pollUpdates() {
    if (!pose_graph_.isInit())
        return;

    int state_idx = pose_graph_.getCurrentStateIdx();
    while (aligned_idx_ < state_idx){
        aligned_idx_++;
        alignFrames(aligned_idx_-frame_interval_, aligned_idx_);
    }
};


void LidarOdometry::alignFrames(int idx0, int idx1) {
    if (!frame_buffer_.hasFrame(idx0) || !frame_buffer_.hasFrame(idx1))
        return;

    // Get frames
    auto frame0 = frame_buffer_.getFrame(idx0);
    auto frame1 = frame_buffer_.getFrame(idx1);

    // Assert a size threshold
    if (frame0.size() < min_frame_size_ || frame1.size() < min_frame_size_)
        return;
    
    // Convert to tensor clouds (shallow copy)
    TensorCloud cloud0 = EigenToTensorCloud(frame0.positions);
    TensorCloud cloud1 = EigenToTensorCloud(frame1.positions);
    
    // Get pose and initial alignment
    Pose3 pose0 = pose_graph_.getPose(idx0);
    Pose3 pose1 = pose_graph_.getPose(idx1);

    Eigen::Matrix4d T_initial = pose0.between(pose1).matrix();

    // Downsample
    auto cloud0_ds = cloud0.VoxelDownSample(voxel_size_).ToLegacy();
    auto cloud1_ds = cloud1.VoxelDownSample(voxel_size_).ToLegacy();

    // Perform ICP alignment
    auto result = open3d::pipelines::registration::RegistrationICP(
        cloud1_ds, cloud0_ds, icp_threshold_, T_initial.matrix(),
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