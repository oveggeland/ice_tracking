#include "mapping/CloudAnalyzer.h"

CloudAnalyzer::CloudAnalyzer(const ros::NodeHandle& nh){
    getParamOrThrow(nh, "/mapping/save_stats", save_stats_);
    if (save_stats_){
        std::string outpath = getParamOrThrow<std::string>(nh, "/outpath");
        std::string stats_path = joinPath(outpath, "cloud_stats/stats.csv");
        makePath(stats_path, true);

        f_out_ = std::ofstream(stats_path);
        f_out_ << "ts,count,freeboard,deformation" << std::endl;
        f_out_ << std::fixed;
    }
};


void CloudAnalyzer::analyzeCloud(double ts, const open3d::t::geometry::PointCloud& pcd){
    CloudStatistics stats;

    // Count
    stats.count = pcd.GetPointPositions().GetShape(0);
    
    // Moments
    auto elevation_tensor = -pcd.GetPointPositions().Slice(1, 2, 3).Flatten();
    auto intensity_tensor = pcd.GetPointAttr("intensities");

    // Freeboard
    stats.freeboard = estimateFreeboard(pcd);

    // Deformation
    stats.deformation = estimateDeformation(pcd);

    if (save_stats_){
        saveStats(ts, stats);
    }
}


float CloudAnalyzer::estimateFreeboard(const open3d::t::geometry::PointCloud& pcd) const{
    return -1;
}

float CloudAnalyzer::estimateDeformation(const open3d::t::geometry::PointCloud& pcd) const{
    if (!pcd.HasPointAttr("deformation"))
        return -1;
    
    return 0.2;
}


void CloudAnalyzer::saveStats(double ts, const CloudStatistics& stats){
    f_out_ << ts;
    f_out_ << "," << stats.count;
    f_out_ << "," << stats.freeboard;
    f_out_ << "," << stats.deformation;
    f_out_ << std::endl;
}