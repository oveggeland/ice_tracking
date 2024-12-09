#include "icetrack/mapping/processing.h"

CloudProcessor::CloudProcessor(){
    // Setup output file
    f_stats_ = std::ofstream("/home/oskar/icetrack/output/stats/stats.csv");
    f_stats_ << "ts,count,z_mean,z_var,z_min,z_max" << std::endl << std::fixed;
}


void CloudProcessor::processCloud(double ts, std::vector<point> points){
    cloud_ = std::move(points); // Assign the new points to the cloud object

    meanElevationFilter();
    calculateMoments();

    f_stats_ << ts << "," << cloud_.size();
    f_stats_ << "," << z_mean_ << "," << z_var_ << "," << z_min_ << "," << z_max_;
    f_stats_ << std::endl;
}


void CloudProcessor::calculateMoments(){
    z_min_ = cloud_[0].z;
    z_max_ = z_min_;
    
    double sum = 0;
    for (auto p: cloud_){
        sum += p.z;

        if (p.z < z_min_)
            z_min_ = p.z;
        else if (p.z > z_max_)
            z_max_ = p.z;
    }
    z_mean_ = sum / cloud_.size();

    sum = 0;
    for (auto p: cloud_){
        double err = p.z - z_mean_;
        sum += err*err;
    }
    z_var_ = sum / cloud_.size();
}

void CloudProcessor::meanElevationFilter(){
    auto z_begin = make_field_iterator<double>(cloud_, 0);
    auto z_end =  make_field_iterator<double>(cloud_, cloud_.size());

    double z_mean = std::accumulate(z_begin, z_end, 0.0) / cloud_.size();

    // Remove out of bounds
    auto it = cloud_.begin();
    for (int idx = 0; idx < cloud_.size(); idx++){
        if (cloud_[idx].z > z_mean + 1 || cloud_[idx].z < z_mean - 2)
            continue;
        else{
            *it = cloud_[idx];
            it++;
        }
    }
    cloud_.resize(std::distance(cloud_.begin(), it));
}

void CloudProcessor::ransacElevationFilter(){
    // RANSAC parameters
    int n_points = 3;
    int n_iterations = 10;

    // Z-value iterators
    auto z_begin = make_field_iterator<double>(cloud_, 0);
    auto z_end =  make_field_iterator<double>(cloud_, cloud_.size());
    
    // Track inliers
    int inlier_count = 0;
    int best_inlier_count = 0;
    std::vector<bool> inliers(cloud_.size());
    std::vector<bool> best_inliers(cloud_.size());

    // Perform RANSAC iterations
    std::vector<double> z_sampled(n_points);
    for (int i = 0; i < n_iterations; i++){
        // Sample random values
        std::sample(z_begin, z_end, z_sampled.begin(), n_points, std::mt19937(std::random_device()()));

        // Find mean
        double mean = std::accumulate(z_sampled.begin(), z_sampled.end(), 0.0) / n_points;

        // Count inliers
        for (int idx = 0; idx < cloud_.size(); idx++){
            double z = cloud_[idx].z;
            if (z < mean - 2 || z > mean + 1)
                inliers[idx] = false;
            else{
                inliers[idx] = true;
                inlier_count ++;
            }
        }

        if (inlier_count > best_inlier_count){
            best_inliers = inliers;
            best_inlier_count = inlier_count;
        }

        inlier_count = 0;
    }

    std::cout << "Number of inliers is: " << best_inlier_count << "/" << cloud_.size() << std::endl;
    // Remove non-inliers
    auto it = cloud_.begin();
    for (int idx = 0; idx < cloud_.size(); idx++){
        if (best_inliers[idx]){
            *it = cloud_[idx];
            it++;
        }
    }
    cloud_.resize(std::distance(cloud_.begin(), it));
}