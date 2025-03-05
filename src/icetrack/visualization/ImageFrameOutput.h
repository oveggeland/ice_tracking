#pragma once

#include <ros/ros.h>

#include "ImageFrame.h"
#include "ImageOutput.h"

#include "utils/ros_params.h"
#include "utils/file_system.h"


class ImageFrameOutput{
public:
    ImageFrameOutput(ros::NodeHandle& nh);
    
    void newImageFrame(const ImageFrame& img_frame) const;
private:
    std::unordered_map<std::string, ImageOutput> image_output_map_;

    void writeProjections(const std::string& label, const ImageFrame& img_frame) const;
    void writeProjectionsBinary(const std::string& label, const ImageFrame& img_frame) const;

    std::string image_folder_; 
    std::string projection_folder_;
};