#include "LidarOdometry.h"
#include "CloudManager.h"


void visualizeAlignment(PointCloudSharedPtr pcd0, PointCloudSharedPtr pcd1, Pose3 T_initial, Pose3 T_align){
    auto pcd1_initial = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_initial.matrix()));
    auto pcd1_aligned = std::make_shared<PointCloud>(PointCloud(*pcd1).Transform(T_align.matrix()));

    pcd0->PaintUniformColor({1, 0, 0});
    pcd1_initial->PaintUniformColor({0, 0, 1});
    pcd1_aligned->PaintUniformColor({0, 0, 1});

    // Offset 
    auto offset = pcd0->GetCenter();
    pcd0->Translate(-offset);
    pcd1_initial->Translate(-offset);
    pcd1_aligned->Translate(-offset);

    open3d::visualization::DrawGeometries({pcd0, pcd1_initial});
    open3d::visualization::DrawGeometries({pcd0, pcd1_aligned});
}

LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, const BatchFixedLagSmoother& smoother) : smoother_(smoother){
    noise_model_ = noiseModel::Diagonal::Sigmas(
        (Vector6() << Vector3::Constant(0.1), Vector3::Constant(2.0)).finished()
    );
}


boost::shared_ptr<gtsam::NonlinearFactor> LidarOdometry::estimateOdometry(int idx1, bool estimate_ice_drift) {
    // Index checking
    if (idx1 < 10)
        return nullptr;
    int idx0 = idx1 - 2;

    // Get current frame cloud and pose
    auto cloud0 = cloud_manager_->getFrame(idx0);
    Pose3 pose0 = smoother_.calculateEstimate<Pose3>(X(idx0));
    double t0 = smoother_.timestamps().at(X(idx0));

    auto cloud1 = cloud_manager_->getFrame(idx1);
    Pose3 pose1 = smoother_.calculateEstimate<Pose3>(X(idx1));
    double t1 = smoother_.timestamps().at(X(idx1));

    Pose3 T_initial = pose0.between(pose1);
    
    if (!cloud0 || !cloud1)
        return nullptr;

    // Assert a size threshold
    if (cloud0->points_.size() < 5000 && cloud1->points_.size() < 5000)
        return nullptr;

    // Downsample
    auto cloud0_ds = cloud0->VoxelDownSample(1.0);
    auto cloud1_ds = cloud1->VoxelDownSample(1.0);

    // Perform ICP alignment
    auto result = open3d::pipelines::registration::RegistrationICP(
        *cloud1_ds, *cloud0_ds, 2.0, T_initial.matrix(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint()
    );
    Pose3 T_align = Pose3(result.transformation_);

    // // Visualize the alignment
    // ROS_INFO_STREAM("FITNESS: " << result.fitness_);
    // visualizeAlignment(cloud0_ds, cloud1_ds, T_initial, T_align);

    // Check if ICP converged
    if (result.fitness_ < 0.2)  // Fitness threshold (tune based on environment)
        return nullptr;

    if (estimate_ice_drift)
        return boost::make_shared<IceOdometryFactor>(X(idx0), X(idx1), D(idx1-1), T_align, t1 - t0, noise_model_);
    else
        return boost::make_shared<BetweenFactor<Pose3>>(X(idx0), X(idx1), T_align, noise_model_);
}