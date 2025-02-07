#include "LidarOdometry.h"


LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, PoseGraph& pose_graph, const FrameBuffer& frame_buffer)
                        : pose_graph_(pose_graph), frame_buffer_(frame_buffer) {
    // Initialize
};

void LidarOdometry::alignFrames() {
    if (!pose_graph_.isInit())
        return;

    int state_idx = pose_graph_.getCurrentStateIdx();
    while (aligned_idx_ < state_idx){
        aligned_idx_++;
        alignFrame(aligned_idx_-1, aligned_idx_);
    }
};


void LidarOdometry::alignFrame(int idx0, int idx1) {
    // Get frames
    auto cloud0 = frame_buffer_.getFrame(idx0);
    auto cloud1 = frame_buffer_.getFrame(idx1);
    if (!cloud0 || !cloud1)
        return;

    // Assert a size threshold
    if (getCloudSize(cloud0) < 5000 || getCloudSize(cloud1) < 5000)
        return;
    
    // Get pose and initial alignment
    Pose3 pose0 = pose_graph_.getPose(idx0);
    Pose3 pose1 = pose_graph_.getPose(idx1);

    Eigen::Matrix4d T_initial = pose0.between(pose1).matrix();

    // Downsample
    auto cloud0_ds = cloud0->ToLegacy().VoxelDownSample(1.0);
    auto cloud1_ds = cloud1->ToLegacy().VoxelDownSample(1.0);

    // Perform ICP alignment
    auto result = open3d::pipelines::registration::RegistrationICP(
        *cloud1_ds, *cloud0_ds, 2.0, T_initial.matrix(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint()
    );
    Eigen::Matrix4d T_align = result.transformation_;

    // Check if ICP converged
    if (result.fitness_ < 0.8)  // Fitness threshold (tune based on environment)
        return;
    //visualizeAlignment(cloud0_ds, cloud1_ds, T_initial, T_align);

    pose_graph_.odometryCallback(idx0, idx1, T_align);
}