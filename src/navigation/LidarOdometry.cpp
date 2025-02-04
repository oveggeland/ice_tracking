#include "LidarOdometry.h"
#include "CloudManager.h"


void visualizeAlignment(PointCloudSharedPtr pcd0, PointCloudSharedPtr pcd1, Pose3 T_initial, Pose3 T_align){
    auto pcd1_initial = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_initial.matrix()));
    auto pcd1_aligned = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_align.matrix()));

    pcd0->PaintUniformColor({1, 0, 0});
    pcd1_initial->PaintUniformColor({0, 0, 1});
    pcd1_aligned->PaintUniformColor({0, 0, 1});

    open3d::visualization::DrawGeometries({pcd0, pcd1_initial});
    open3d::visualization::DrawGeometries({pcd0, pcd1_aligned});
}

LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, const BatchFixedLagSmoother& smoother) : smoother_(smoother){
    noise_model_ = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector3::Constant(0.1), Vector3::Constant(2.0)).finished()
    );
}

boost::shared_ptr<gtsam::NonlinearFactor> LidarOdometry::createOdometryFactor(int idx0, int idx1, Pose3 T_align){
    ROS_INFO_STREAM("Building odometry factor between poses " << idx0 << " and " << idx1);
    return boost::make_shared<OdometryFactor>(X(idx0), X(idx1), T_align, noise_model_);
};

boost::shared_ptr<gtsam::NonlinearFactor> LidarOdometry::estimateOdometry(int idx1) {
    // Index checking
    if (idx1 < 10)
        return nullptr;
    int idx0 = idx1 - 1;

    // Get pose and establish initial transformation
    Pose3 pose0 = smoother_.calculateEstimate<Pose3>(X(idx0));
    Pose3 pose1 = smoother_.calculateEstimate<Pose3>(X(idx1));
    Pose3 T_initial = pose0.between(pose1);
    
    // Get/check cloud frames
    auto frame0 = cloud_manager_->getFrame(idx0);
    auto frame1 = cloud_manager_->getFrame(idx1);
    if (!frame0 || !frame1)
        return nullptr;

    // Assert a size threshold
    if (frame0->points_.size() < 10000 && frame1->points_.size() < 10000)
        return nullptr;

    // Downsample
    auto frame0_ds = frame0->VoxelDownSample(0.5);
    auto frame1_ds = frame1->VoxelDownSample(0.5);
    //frame0 = frame0->VoxelDownSample(0.5);
    //frame1 = frame1->VoxelDownSample(0.5);

    // Convert Pose3 to Eigen::Matrix4d for Open3D
    Eigen::Matrix4d T_init_eigen = T_initial.matrix();

    // Perform ICP alignment
    auto result = open3d::pipelines::registration::RegistrationICP(
        *frame1_ds, *frame0_ds, 1.0, T_initial.matrix(), // 0.5 is distance threshold (tune as needed)
        open3d::pipelines::registration::TransformationEstimationPointToPoint()
        //open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 50) // Max 30 iterations
    );
    Pose3 T_align = Pose3(result.transformation_);

    // Check if ICP converged
    if (result.fitness_ < 0.2)  // Fitness threshold (tune based on environment)
        return nullptr;

    // Visualize the alignment
    // visualizeAlignment(frame0_ds, frame1_ds, T_initial, T_align);
    
    return createOdometryFactor(idx0, idx1, T_align);
}