#include "OdometryEstimator.h"


OdometryEstimator::OdometryEstimator(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer)
                        : pose_graph_(pose_graph), frame_buffer_(frame_buffer) {
    // Import config
    getParamOrThrow(nh, "/lidar_odometry/enabled", enabled_);
    getParamOrThrow(nh, "/lidar_odometry/key_frame_interval", key_frame_interval_);
    getParamOrThrow(nh, "/lidar_odometry/min_frame_size", min_frame_size_);
    getParamOrThrow(nh, "/lidar_odometry/voxel_size", voxel_size_);
    getParamOrThrow(nh, "/lidar_odometry/icp_max_iter", icp_max_iter_);
    getParamOrThrow(nh, "/lidar_odometry/icp_relative_rmse", icp_relative_rmse_);
    getParamOrThrow(nh, "/lidar_odometry/icp_relative_fitness", icp_relative_fitness_);
    getParamOrThrow(nh, "/lidar_odometry/icp_threshold", icp_threshold_);
    getParamOrThrow(nh, "/lidar_odometry/icp_min_fitness", icp_min_fitness_);
};

void OdometryEstimator::estimateOdometry(int idx1) {
    if (!enabled_ || (idx1 - prev_idx_ < key_frame_interval_))
        return;
    
    // Temporary storage of previous index and cloud
    const int idx0 = prev_idx_;
    const PointCloudPtr cloud0 = prev_cloud_;

    // Reset prev values
    prev_idx_ = idx1;
    prev_cloud_ = nullptr;

    // Get new frame, assert size and downsample cloud
    auto frame1 = frame_buffer_.getFrame(idx1);
    if (!frame1 || frame1->size() < min_frame_size_)
        return; // Frame not big enough...
    const PointCloudPtr cloud1 = frame1->local().VoxelDownSample(voxel_size_);
    prev_cloud_ = cloud1; 

    // Check that both clouds exist
    if (!cloud0 || !cloud1)
        return; 

    // Get pose and estimate initial alignment
    gtsam::Pose3 pose0, pose1;
    if (!pose_graph_.poseQuery(idx0, pose0) || !pose_graph_.poseQuery(idx1, pose1))
        return; // Pose query failed...
    Eigen::Matrix4d T_initial = pose0.between(pose1).matrix();


    // ICP
    auto result = open3d::pipelines::registration::RegistrationICP(
        *cloud1, *cloud0, icp_threshold_, T_initial,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(),
        open3d::pipelines::registration::ICPConvergenceCriteria(icp_relative_fitness_, icp_relative_rmse_, icp_max_iter_)
    ); 
    Eigen::Matrix4d T_align = result.transformation_;


    // Check if ICP converged
    if (result.fitness_ < icp_min_fitness_)  // Fitness threshold (tune based on environment)
        return;

    // visualizeAlignment(*cloud0, *cloud1, T_initial, T_align);
    pose_graph_.odometryCallback(idx0, idx1, T_align);
}