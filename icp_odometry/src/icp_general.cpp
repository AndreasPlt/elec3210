#include "icp_general.h"
#include "parameters.h"

#include <string>
#include <unordered_set>



// icp_general constructor
template<class S, class T>
icp_general<S, T>::icp_general(): src_cloud_transformed(new pcl::PointCloud<S>) {
    std::unordered_set<std::string> valid_weight_modes = {"distance", "uniform"};
    std::unordered_set<std::string> valid_reject_modes = {"threshold", "percentage", "none"};

    // check if weight_mode is valid
    if (valid_weight_modes.count(params::weight_mode) == 0) {
        std::cout << "Invalid weight_mode" << std::endl;
        exit(1);
    }
    // check if reject_mode is valid
    if (valid_reject_modes.count(params::reject_mode) == 0) {
        std::cout << "Invalid reject_mode" << std::endl;
        exit(1);
    }
    // check if reject_threshold is valid
    if (params::reject_threshold <= 0) {
        std::cout << "Invalid reject_threshold" << std::endl;
        exit(1);
    }
    // check if reject_percentage is valid
    if (params::reject_percentage < 0 || params::reject_percentage > 1) {
        std::cout << "Invalid reject_percentage" << std::endl;
        exit(1);
    }
    // check if max_iterations is valid
    if (params::max_iterations <= 0) {
        std::cout << "Invalid max_iterations" << std::endl;
        exit(1);
    }
    // check if max_distance is valid
    if (params::max_distance <= 0) {
        std::cout << "Invalid max_distance" << std::endl;
        exit(1);
    }
    // check if transformation_epsilon is valid
    if (params::transformation_epsilon <= 0) {
        std::cout << "Invalid transformation_epsilon" << std::endl;
        exit(1);
    }
}

template<class S, class T>
void icp_general<S, T>::align(Eigen::Matrix4d init_guess) {
    // initialize current_transformation
    current_transformation = init_guess;
    pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, current_transformation);

    // subsample clouds?
    determine_corresponding_points();
    double prev_error = calculate_error();
    std::cout << "Initial error: " << prev_error << std::endl;
    Eigen::Matrix4d prev_transformation = current_transformation;

    for (int i = 0; i < params::max_iterations; i++) {
        std::cout << "Iteration: " << i << std::endl;

        // subsample clouds?
        // determine corresponding points
        determine_corresponding_points();

        // weight/reject pairs
        if (params::weight_mode == "distance") {
            weight_pairs_distance();
        }

        if (params::reject_mode == "threshold") {
            reject_pairs_threshold();
        }
        else if (params::reject_mode == "percentage") {
            reject_pairs_trimming();
        }

        // compute translation and rotation
        calculate_rotation();
        
        // apply R and t to all points
        pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, current_transformation);

        // compute error
        double error = calculate_error();

        // check convergence
        if (error > prev_error) {
            current_transformation = prev_transformation;
            std::cout << "Error increased, reverting to previous transformation" << std::endl;
            break;
        }
        if (prev_error - error < params::transformation_epsilon) {
            std::cout << "Error converged" << std::endl;
            break;
        }
        prev_transformation = current_transformation;
        prev_error = error;
    }
    final_transformation = current_transformation;
}

/**
 * @brief Determines the corresponding points between the source and target clouds.
 * 
 * This function uses a KDTree to find the nearest point in the target cloud to each point in the source cloud.
 * It then stores the corresponding points in a vector of correspondence pairs.
 * 
 * @return None.
 */
template<class S, class T>
void icp_general<S, T>::determine_corresponding_points() {
    // reset correspondence pairs
    correspondence_pairs.clear();
    
    // for each point in source cloud
    for (const auto& point: src_cloud_transformed->points) {
        // find nearest point in target cloud
        const T nearest_point = get_nearest_point(point);
        // add pair to corresponding points
        struct correspondence_pair<S, T> pair(point, nearest_point);
        pair.distance = pcl::squaredEuclideanDistance(point, nearest_point);
        correspondence_pairs.push_back(pair);
    }
}

/**
 * @brief Obtain the nearest point in the target cloud to a given point.
 * 
 * This function uses a KDTree to find the nearest point in the target cloud to a given point.
 * @param point Point to find nearest point to.
 * @return pcl::PointXYZ Nearest point in target cloud.
 */
template<class S, class T>
T icp_general<S, T>::get_nearest_point(S point) {
        T nearest_point;
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        
        // TODO add a new point to search with
        tar_kdtree.nearestKSearch(point, 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        nearest_point = tar_cloud->points[pointIdxKNNSearch[0]];

        // TODO: multiple nearest points
        // radius search
        // check consistency
        return nearest_point;
    }

/**
 * \brief Rejects certain percentage of correspondence pairs based on their distance.
 *
 * This function calculates the distance for each correspondence pair and sorts them
 * based on distance. It then rejects the last percentage of pairs. 
 * 
 *
 * \param percentage The percentage of pairs to reject.
 * \return None.
 */
template<class S, class T>
void icp_general<S, T>::reject_pairs_trimming() {
    // load percentage
    double percentage = params::reject_percentage;

    //sort the pairs by distance using a lambda function
    std::sort(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const correspondence_pair<S, T>& pair1, const correspondence_pair<S, T>& pair2) {
                  return pair1.distance < pair2.distance;
              });
    // leave the weight of the first num_pairs, set rest to zero
    int num_pairs = correspondence_pairs.size() * (1 - percentage);
    for (int i = num_pairs; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].weight = 0;
    }
}

/**
 * @brief Rejects correspondence pairs based on a threshold distance.
 * 
 * This function calculates the distance for each correspondence pair and rejects
 * pairs that have a distance greater than the threshold.
 * 
 * @param threshold The threshold distance.
 * @return None.
 */
template<class S, class T>
void icp_general<S, T>::reject_pairs_threshold(){
    // load threshold
    double threshold = params::reject_threshold;

    // reject pairs with distance greater than threshold
    for (auto &pair: correspondence_pairs) {
        if (pair.distance > threshold) {
            pair.weight = 0;
        }
    }
}
/**
 * \brief Assigns weights to correspondence pairs based on their distance.
 *
 * This function calculates the maximum distance among all correspondence pairs,
 * and then assigns weights to each pair based on their distance relative to the maximum.
 * The weight is determined by subtracting the pair's distance from the maximum distance,
 * and dividing it by the maximum distance.
 *
 * \return None.
 */
template<class S, class T>
void icp_general<S, T>::weight_pairs_distance() {
    struct correspondence_pair<S, T> max_pair = *std::max_element(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const struct correspondence_pair<S, T>& pair1, const struct correspondence_pair<S, T>& pair2) {
                  return pair1.distance < pair2.distance;
              });
    int max_val = max_pair.distance;
    for (auto &pair: correspondence_pairs) {
        pair.weight = 1 - pair.distance / max_val;
    }
}

template<class S, class T>
std::pair<pcl::PointXYZ, pcl::PointXYZ> icp_general<S, T>::calculate_means() {
    double src_sumX = 0.0, src_sumY = 0.0, src_sumZ = 0.0,
        tar_sumX = 0.0, tar_sumY = 0.0, tar_sumZ = 0.0;
    double total_weight;

    for (const auto& pair: correspondence_pairs) {
        src_sumX += pair.src_point.x * pair.weight;
        src_sumY += pair.src_point.y * pair.weight;
        src_sumZ += pair.src_point.z * pair.weight;
        tar_sumX += pair.tar_point.x * pair.weight;
        tar_sumY += pair.tar_point.y * pair.weight;
        tar_sumZ += pair.tar_point.z * pair.weight;
        total_weight += pair.weight;
    }

    double src_meanX = src_sumX / total_weight;
    double src_meanY = src_sumY / total_weight;
    double src_meanZ = src_sumZ / total_weight;
    double tar_meanX = tar_sumX / total_weight;
    double tar_meanY = tar_sumY / total_weight;
    double tar_meanZ = tar_sumZ / total_weight;

    pcl::PointXYZ src_mean(src_meanX, src_meanY, src_meanZ);
    pcl::PointXYZ tar_mean(tar_meanX, tar_meanY, tar_meanZ);

    return std::make_pair(src_mean, tar_mean);
}


template class icp_general<pcl::PointXYZ, pcl::PointXYZ>;
template class icp_general<pcl::PointXYZ, pcl::PointNormal>;