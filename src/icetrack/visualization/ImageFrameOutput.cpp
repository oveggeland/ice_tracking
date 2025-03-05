#include "ImageFrameOutput.h"

ImageFrameOutput::ImageFrameOutput(ros::NodeHandle& nh) {
    const std::string ws = getParamOrThrow<std::string>(nh, "/workspace");
    const std::string exp = getParamOrThrow<std::string>(nh, "/exp");
    const std::string output_folder = joinPaths({ws, exp, "image_frames"});

    image_folder_ = joinPaths({output_folder, "images/"});
    makePath(image_folder_, true); // Purge 

    projection_folder_ = joinPaths({output_folder, "projections/"});
    makePath(projection_folder_, true); // Purge 


    // Setup raw image output
    const std::string raw_image_folder = joinPaths({image_folder_, "raw/"});
    makePath(raw_image_folder, true);
    image_output_map_["raw"] = ImageOutput(nh, raw_image_folder, "", 10);

    // Setup elevation image output
    // const std::string elev_imposed_image_folder = joinPaths({image_folder_, "elev/"});
    // makePath(elev_imposed_image_folder, true);
    // image_output_map_["elev"] = ImageOutput(nh, elev_imposed_image_folder, "", 10);
};
        
void ImageFrameOutput::newImageFrame(const ImageFrame& img_frame) const{
    const double ts = img_frame.getStamp();

    // Make label
    std::string label = std::to_string(static_cast<long>(1000 * ts));

    // Check what images we want to output
    if (image_output_map_.find("raw") != image_output_map_.end()){
        image_output_map_.at("raw").newImage(label, ts, img_frame.getRawImage());
    }

    // Check what images we want to output
    if (image_output_map_.find("elev") != image_output_map_.end()){
        image_output_map_.at("elev").newImage(label, ts, img_frame.getImposedElevationImage());
    }

    bool save_projections_ = true;
    if (save_projections_)
        writeProjectionsBinary(label, img_frame);
}





void ImageFrameOutput::writeProjections(const std::string& label, const ImageFrame& img_frame) const {
    std::string out_file = joinPaths({projection_folder_, label+".csv"});
    std::ofstream f_out(out_file);
    f_out << "u,v,elevation,intensity,timediff" << std::endl << std::fixed;
    
    const auto& inliers = img_frame.inliers();
    const auto& uv = img_frame.uv();
    const auto& elev = img_frame.elevation();
    const auto& intensity = img_frame.intensity();
    const auto& dt = img_frame.dt();

    for (const int& idx: inliers){
        f_out << uv(0, idx) << "," << uv(1, idx) << "," << elev(idx) << "," << intensity(idx) << "," << dt(idx) << std::endl;
    }
    f_out.close();
}

void ImageFrameOutput::writeProjectionsBinary(const std::string& label, const ImageFrame& img_frame) const {
    std::string out_file = joinPaths({projection_folder_, label + ".bin"});
    
    // Open the output file in binary mode
    std::ofstream f_out(out_file, std::ios::binary);
    if (!f_out) {
        throw std::ios_base::failure("Failed to open file for writing");
    }

    const auto& inliers = img_frame.inliers();
    const auto& uv = img_frame.uv();
    const auto& elev = img_frame.elevation();
    const auto& intensity = img_frame.intensity();
    const auto& dt = img_frame.dt();

    // Write the number of inliers as a header (can be used for reading)
    size_t num_inliers = inliers.size();
    f_out.write(reinterpret_cast<const char*>(&num_inliers), sizeof(num_inliers));

    // Write the data
    for (const int& idx : inliers) {
        // Write u, v, elevation, intensity, and time difference as binary
        float u = uv(0, idx);
        float v = uv(1, idx);
        float elev_value = elev(idx);
        float intensity_value = intensity(idx);
        float dt_value = dt(idx);

        // Writing each of the values in the order: u, v, elevation, intensity, timediff
        f_out.write(reinterpret_cast<const char*>(&u), sizeof(u));
        f_out.write(reinterpret_cast<const char*>(&v), sizeof(v));
        f_out.write(reinterpret_cast<const char*>(&elev_value), sizeof(elev_value));
        f_out.write(reinterpret_cast<const char*>(&intensity_value), sizeof(intensity_value));
        f_out.write(reinterpret_cast<const char*>(&dt_value), sizeof(dt_value));
    }

    f_out.close();
}