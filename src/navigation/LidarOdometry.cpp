#include "navigation/LidarOdometry.h"

void visualizeCloud(const open3d::geometry::PointCloud& cloud) {
    open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(cloud)}, 
                                          "Lidar Point Cloud", 
                                          800, 600);
}

void visualizeAlignment(std::shared_ptr<open3d::geometry::PointCloud> cloud0,
                        std::shared_ptr<open3d::geometry::PointCloud> cloud1,
                        Pose3 T_align) {
    if (!cloud0 || !cloud1) {
        std::cerr << "Error: One of the input clouds is null!" << std::endl;
        return;
    }

    // Create copies of the clouds to avoid modifying the original data
    auto cloud0_vis = std::make_shared<open3d::geometry::PointCloud>(*cloud0);
    auto cloud1_vis = std::make_shared<open3d::geometry::PointCloud>(*cloud1);

    // Apply different colors for distinction
    cloud0_vis->PaintUniformColor(Eigen::Vector3d(1, 0, 0));  // Red for cloud0
    cloud1_vis->PaintUniformColor(Eigen::Vector3d(0, 1, 0));  // Green for cloud1 (before transformation)
    
    // Transform cloud
    cloud1_vis->Transform(T_align.matrix());

    // Visualize both clouds together
    open3d::visualization::DrawGeometries({cloud0_vis, cloud1_vis}, 
                                          "ICP Alignment Visualization", 
                                          800, 600);
}

LidarOdometry::LidarOdometry(const ros::NodeHandle& nh, const LidarFrameBuffer& frame_buffer) : frame_buffer_(frame_buffer){
}


bool LidarOdometry::estimateOdometry(int idx1) {
    // Find frames to align
    int idx0 = idx1 - frame_interval_;
    
    // Get frames
    auto frame0 = frame_buffer_.getFrame(idx0);
    auto frame1 = frame_buffer_.getFrame(idx1);
    if (!frame0 || !frame1)
        return false;

    return true;
    // // Extract initial pose estimate
    // Pose3 pose0 = frame0->second.pose;
    // Pose3 pose1 = frame1->second.pose;
    // Pose3 T_initial = pose0.between(pose1); // Initial guess

    // // Extract point clouds
    // std::shared_ptr<open3d::geometry::PointCloud> cloud0 = frame0->second.pcd;
    // std::shared_ptr<open3d::geometry::PointCloud> cloud1 = frame1->second.pcd;
    // if (!cloud0 || !cloud1)
    //     return false; // Nullptrs

    // // ICP registration
    // auto result_icp = open3d::pipelines::registration::RegistrationICP(
    //     *cloud1, *cloud0, icp_threshold_, T_initial.matrix(),
    //     open3d::pipelines::registration::TransformationEstimationPointToPoint()
    //     // open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 50) // Max 50 iterations
    // );

    // // Extract final transformation
    // Pose3 T_icp(result_icp.transformation_);

    // ROS_WARN_STREAM("T_initial: " << T_initial.translation().transpose());
    // ROS_WARN_STREAM("T_icp: " << T_icp.translation().transpose());
    
    // visualizeAlignment(cloud0, cloud1, T_initial);
    // visualizeAlignment(cloud0, cloud1, T_icp);

    // if ((T_icp.translation() - T_initial.translation()).norm() < 5){
    //     ROS_INFO("Sucess");
    //     return true;
    // }
    // ROS_WARN("Failed");
    // return false; // Return ICP-refined transformation
}
