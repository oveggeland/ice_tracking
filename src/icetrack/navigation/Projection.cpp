#include "icetrack/navigation/Projection.h"

Projection::Projection(){}

Projection::Projection(ros::NodeHandle nh){
    getParamOrThrow(nh, "/navigation/crs_source", crs_source_);
    getParamOrThrow(nh, "/navigation/crs_target", crs_target_);

    projection_ = proj_create_crs_to_crs(proj_context_create(), crs_source_.c_str(), crs_target_.c_str(), NULL);
}

gtsam::Point2 Projection::project(double latitude, double longitude) const{
    PJ_COORD input_coords = proj_coord(latitude, longitude, 0, 0);
    PJ_COORD output_coords = proj_trans(projection_, PJ_FWD, input_coords);
    
    return gtsam::Point2(output_coords.xy.y, output_coords.xy.x);
}