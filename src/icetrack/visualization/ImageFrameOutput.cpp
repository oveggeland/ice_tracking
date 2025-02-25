#include "ImageFrameOutput.h"

ImageFrameOutput::ImageFrameOutput(ros::NodeHandle& nh) {
    image_output_map_["raw"] = ImageOutput(nh, "", "/viz/raw", 10);
};
        
void ImageFrameOutput::newImageFrame(const ImageFrame& img_frame) const{
    const double ts = img_frame.getStamp();

    // Check what images we want to output
    if (image_output_map_.find("raw") != image_output_map_.end()){
        image_output_map_.at("raw").newImage(ts, img_frame.getRawImage());
    }
}