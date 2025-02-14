#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


struct DatasetSource{
    double t_img;
    cv::Mat img;

    std::vector<double> timestamps;
    std::vector<double> intensities;
    Eigen::Matrix3Xf r_world;
};



/*
This class is used to generate a lidar/image dataset, where pointclouds are 
aligned with the images to create a multimodal depth map of the scene.

In: 
- Image w/
-- Timestamp

- PointCloud w/
-- Timestamps
-- Point vector (nav frame)
-- Intensities

- Camera object (Camera.h)

Out (to file):
- Image png
- Binary array of projected values
*/
class DatasetGenerator{
public:
    DatasetGenerator(ros::NodeHandle& nh){};

    
};