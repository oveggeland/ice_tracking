#pragma once

#include <ros/ros.h>

#include "ImageFrame.h"
#include "ImageOutput.h"

#include "utils/ros_params.h"


class ImageFrameOutput{
public:
    ImageFrameOutput(ros::NodeHandle& nh);
    
    void newImageFrame(const ImageFrame& img_frame) const;
private:
    std::unordered_map<std::string, ImageOutput> image_output_map_;
};