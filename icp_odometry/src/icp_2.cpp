#include "icp_2.h"
#include "parameters.h"

#include <string>
#include <unordered_set>
#include <assert.h>     /* assert */

// constructor
icp_2::icp_2():
    src_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>), 
    src_cloud(new pcl::PointCloud<pcl::PointXYZ>), 
    tar_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
}

// setter functions
void icp_2::setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud){
    this->src_cloud = _src_cloud;
    pcl::copyPointCloud(*src_cloud, *src_cloud_transformed);
}

void icp_2::setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud){
    this->tar_cloud = _tar_cloud;
    this->tar_tree.setInputCloud(this->tar_cloud);
}

// getter functions
Eigen::Matrix4d icp_2::getFinalTransformation(){
    return final_transformation;
}

double icp_2::getError(){
    pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, getFinalTransformation());
    determine_corresponding_points();
    weight_pairs();
    reject_pairs();
    return _get_error();
}


// private functions
void icp_2::determine_corresponding_points(){
    correspondence_pairs.clear();

    for(const auto& src_point: src_cloud_transformed->points){
        std::vector<int> indices;
        std::vector<float> distances;
        const int num_nearest_neighbors = 1;
        
        tar_tree.nearestKSearch(src_point, num_nearest_neighbors, indices, distances);
        pcl::PointXYZ tar_point = (*tar_cloud)[indices[0]];

        correspondence_pair_2 pair(src_point, tar_point);
        pair.distance = distances[0];

        correspondence_pairs.push_back(pair);
    }
}

Eigen::Matrix4d icp_2::compute_transformation() {
    // compute means of the point clouds
    double src_mean_x = 0.0, src_mean_y = 0.0, src_mean_z = 0.0,
              tar_mean_x = 0.0, tar_mean_y = 0.0, tar_mean_z = 0.0;
    double total_weight = 0.0;

    for(const auto& pair: correspondence_pairs){
        src_mean_x += pair.src_point.x * pair.weight;
        src_mean_y += pair.src_point.y * pair.weight;
        src_mean_z += pair.src_point.z * pair.weight;

        tar_mean_x += pair.tar_point.x * pair.weight;
        tar_mean_y += pair.tar_point.y * pair.weight;
        tar_mean_z += pair.tar_point.z * pair.weight;

        total_weight += pair.weight;
    }
    src_mean_x /= total_weight;
    src_mean_y /= total_weight;
    src_mean_z /= total_weight;
    tar_mean_x /= total_weight;
    tar_mean_y /= total_weight;
    tar_mean_z /= total_weight;


    // compute cross covariance matrix
    Eigen::Matrix3d cross_covariance_matrix;
    cross_covariance_matrix.setZero();
    for(const auto& pair: correspondence_pairs){
        cross_covariance_matrix(0, 0) += (pair.src_point.x - src_mean_x) * (pair.tar_point.x - tar_mean_x) * pair.weight;
        cross_covariance_matrix(0, 1) += (pair.src_point.x - src_mean_x) * (pair.tar_point.y - tar_mean_y) * pair.weight;
        cross_covariance_matrix(0, 2) += (pair.src_point.x - src_mean_x) * (pair.tar_point.z - tar_mean_z) * pair.weight;
        cross_covariance_matrix(1, 0) += (pair.src_point.y - src_mean_y) * (pair.tar_point.x - tar_mean_x) * pair.weight;
        cross_covariance_matrix(1, 1) += (pair.src_point.y - src_mean_y) * (pair.tar_point.y - tar_mean_y) * pair.weight;
        cross_covariance_matrix(1, 2) += (pair.src_point.y - src_mean_y) * (pair.tar_point.z - tar_mean_z) * pair.weight;
        cross_covariance_matrix(2, 0) += (pair.src_point.z - src_mean_z) * (pair.tar_point.x - tar_mean_x) * pair.weight;
        cross_covariance_matrix(2, 1) += (pair.src_point.z - src_mean_z) * (pair.tar_point.y - tar_mean_y) * pair.weight;
        cross_covariance_matrix(2, 2) += (pair.src_point.z - src_mean_z) * (pair.tar_point.z - tar_mean_z) * pair.weight;
    }

    // compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // compute rotation matrix
    Eigen::Matrix3d R = V * U.transpose();
    //Eigen::Matrix3d R = U * V.transpose();

    // compute translation vector
    Eigen::Vector3d t = Eigen::Vector3d(tar_mean_x, tar_mean_y, tar_mean_z) 
        - R * Eigen::Vector3d(src_mean_x, src_mean_y, src_mean_z);

    // compute transformation matrix
    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    return transformation;
}

double icp_2::_get_error(Eigen::Matrix4d iter_transformation){
    double error = 0.0;
    Eigen::Affine3d transformation_affine;
    transformation_affine.matrix() = iter_transformation;

    for (const auto& pair: correspondence_pairs) {
        const pcl::PointXYZ transformed_point = pcl::transformPoint(pair.src_point, transformation_affine);
        error += pair.weight * pcl::squaredEuclideanDistance(pair.tar_point, transformed_point);
    }

    return error;
}


// weighting functions 
void icp_2::weight_pairs() {
    switch(params::weight_mode){
        case params::UNIFORM:
            break;
        case params::DISTANCE_MAX_SCALING:
            weight_pairs_max_dist_scaling();
            break;
        case params::DISTANCE_INVERSE:
            weight_pairs_distance_inverse();
            break;
        default:
            std::cout << "Invalid weighting mode. No weighting applied." << std::endl;
            break;
    }
}

void icp_2::weight_pairs_max_dist_scaling(){
    // calculate max distance
    struct correspondence_pair_2 max_pair = *std::max_element(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const struct correspondence_pair_2& pair1, const struct correspondence_pair_2& pair2) {
                  return pair1.distance < pair2.distance;
              });
    double max_val = max_pair.distance;

    // scale weights
    for (auto &pair: correspondence_pairs) {
        pair.weight = 1 - (pair.distance / max_val);
    }
}

void icp_2::weight_pairs_distance_inverse() {
    for (auto &pair: correspondence_pairs) {
        pair.weight = 1 / (pair.distance + 1);
    }
}

// rejecting functions
void icp_2::reject_pairs() {
    switch(params::reject_mode){
        case params::NONE:
            break;
        case params::THRESHOLD:
            reject_pairs_threshold();
            break;
        case params::PERCENTAGE:
            reject_pairs_percentage();
            break;
        default:
            std::cout << "Invalid rejection mode. No rejection applied." << std::endl;
            break;
    }
}

void icp_2::reject_pairs_threshold(){
    // load threshold
    for (auto &pair: correspondence_pairs) {
        if (pair.distance > params::max_distance) {
            pair.weight = 0;
        }
    }
}

void icp_2::reject_pairs_percentage(){
    // load percentage
    double percentage = params::reject_percentage;

    // sort the pairs by distance using a lambda function
    std::sort(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const correspondence_pair_2& pair1, const correspondence_pair_2& pair2) {
                  return pair1.distance < pair2.distance;
              });
    
    // leave the weight of the first num_pairs, set rest to zero
    int num_pairs = correspondence_pairs.size() * (1 - percentage);
    for (int i = num_pairs; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].weight = 0;
    }
}

// public function
void icp_2::align(pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4d init_guess){
    Eigen::Matrix4d iter_transformation;

    final_transformation = init_guess;

    pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, init_guess);

    double prev_error = std::numeric_limits<double>::max();
    double error = 0;

    int i = 0;
    std::string break_reason = "All iterations completed";
    for(int i = 0; i < params::max_iterations; i++){
        pcl::transformPointCloud(*src_cloud, *src_cloud_transformed, final_transformation);

        determine_corresponding_points();
        weight_pairs();
        reject_pairs();
        iter_transformation = compute_transformation();
        error = _get_error(iter_transformation);

        // check convergence criteria
        if (error > prev_error){
            break_reason = "Error increased";
            break;
        }
        if (std::abs(error - prev_error) < params::transformation_epsilon){
            break_reason = "Error converged";
            break;
        }
        // update error and transformation
        prev_error = error;
        final_transformation = iter_transformation * final_transformation;
    }
    std::cout << "Statistics:\t" << "Error: " << error << "\tIterations: " << i << std::endl;

    pcl::transformPointCloud(*src_cloud, *output, final_transformation);
}

