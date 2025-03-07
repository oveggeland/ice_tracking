#include "frontend/Floe.h"



Floe::expandCluster(const int idx){
    // 
}

/*
Classify all points as noise, edge or core (as in DBSCAN). 
Return list of noise indices.
*/
Floe::classifyPoints(){
    // Reset class labels
    const int n_points = size();
    point_class_.resize(n_points, 0); // Default class is 0 (noise)

    std::vector<int> indices;
    std::vector<double> distance2;
    for (int i = 0; i < n_points; ++i){
        const Eigen::Vector3d p_query = cloud_.points[i];

        // tree.SearchKNN(p_query, 10, indices, distance2);
        
        // double r2_max = *std::max_element(distance2.begin(), distance2.end());

        // if (r2_max < 1.0){
        //     // Found a core point, start expanding
        //     expandCluster(i);
        //     break;
        // }
    }
}