#ifndef ICP_IMPL_H
#define ICP_IMPL_H

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include "parameters.h"
#include "icp_impl.h"


// icp_implementation constructor
icp_implementation::icp_implementation() {}

determine_corresponding_points(pcl::PointCloud<pcl::PointXYZ> &src_cloud, pcl::PointCloud<pcl::PointXYZ> &tar_cloud) {
    // build kd-tree for target cloud
    tar_kdtree.setInputCloud(tar_cloud);
    // for each point in source cloud
    for (int i = 0; i < src_cloud->points.size(); i++) {
        // find nearest point in target cloud
        pcl::PointXYZ nearest_point = getNearestPoint(src_cloud->points[i]);
        // add pair to corresponding points
        correspondence_pairs.push_back(std::make_pair(src_cloud->points[i], nearest_point));
    }
}


icp_implementation::getNearestPoint(pcl::PointXYZ point) {
        pcl::PointXYZ nearest_point;
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        tar_kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        nearest_point = tar_cloud->points[pointIdxNKNSearch[0]];

        // TODO: multiple nearest points
        // radius search
        // check consistency
        return nearest_point;
    }
/**
 * \brief Assigns weights to correspondence pairs based on their distance.
 *
 * This function calculates the distance for each correspondence pair and sorts them
 * based on distance. It then assigns weights to pairs, where the first num_pairs will
 * have a weight of 1, and the rest will have a weight of 0.
 *
 * \param percentage The percentage of pairs to assign weight 1 (range: 0.0 to 1.0).
 * \return None.
 */
void icp_implementation::reject_pairs_trimming(float percentage) {
    //calculate the distance of each pair
    for (int i = 0; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].distance = calculate_distance(correspondence_pairs[i]);
    }
    //sort the pairs by distance using a lambda function
    std::sort(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const correspondence_pair& pair1, const correspondence_pair& pair2) {
                  return pair1.distance < pair2.distance;
              });
    // set the weight of the first num_pairs to 1, rest to zero
    int num_pairs = correspondence_pairs.size() * (1 - percentage);
    for (int i = 0; i < num_pairs; i++) {
        correspondence_pairs[i].weight = 1;
    }
    for (int i = num_pairs; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].weight = 0;
    }
}
void icp_implementation::reject_pairs_threshold(float threshold){
    //calculate the distance of each pair
    for (int i = 0; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].distance = calculate_distance(correspondence_pairs[i]);
    }
    for (int i = 0; i<correspondence_pairs.size(); i++){
        if (correspondence_pairs[i].distance > threshold){
            correspondence_pairs[i].weight = 0;
        }
        else{
            correspondence_pairs[i].weight = 1;
        }
    }
}
void icp_implementation::weight_pairs(){
    struct correspondence_pair max_pair = std::max_element(correspondence_pairs.begin(), correspondence_pairs.end(),
              [](const correspondence_pair& pair1, const correspondence_pair& pair2) {
                  return pair1.distance < pair2.distance;
              });
    int max_val = max_element->distance;
    for (int i = 0; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].weight = 1 - correspondence_pairs[i].distance/max_val;
    }
}

icp_implementation::align(pcl::PointCloud<pcl::PointXYZ> &output_cloud, Eigen::Matrix4d init_guess) {
    // determine corresponding points
    determine_corresponding_points();
    // weight/reject pairs
    weight_pairs();
    reject_pairs();
    // compute translation and rotation
    Eigen::Matrix4d R = calculate_rotation_point2point();
    // apply R and t to all points
    
    pcl::transformPointCloud(*src_cloud, output_cloud, R);
    // compute error
    
    // check convergence (error > threshold)
        // if not converged, repeat
    // output final alignment
}

icp_implementation::calculate_rotation_point2point() {

    // compute means of src and tar clouds
    double src_sumX = 0.0d, src_sumY = 0.0d, src_sumZ = 0.0d,
        tar_sumX = 0.0d, tar_sumY = 0.0d, tar_sumZ = 0.0d;
    double total_weight;

    for(const auto& pair: correspondence_pairs) {
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
    
    // compute cross covariance matrix
    Eigen::Matrix3d cross_covariance_matrix;
    cross_covariance_matrix.setZero();
    for(const &auto pair: correspondence_pairs) {
        cross_covariance_matrix(0, 0) += (pair.src_point.x - src_meanX) * (pair.tar_point.x - tar_meanX) * pair.weight;
        cross_covariance_matrix(0, 1) += (pair.src_point.x - src_meanX) * (pair.tar_point.y - tar_meanY) * pair.weight;
        cross_covariance_matrix(0, 2) += (pair.src_point.x - src_meanX) * (pair.tar_point.z - tar_meanZ) * pair.weight;
        cross_covariance_matrix(1, 0) += (pair.src_point.y - src_meanY) * (pair.tar_point.x - tar_meanX) * pair.weight;
        cross_covariance_matrix(1, 1) += (pair.src_point.y - src_meanY) * (pair.tar_point.y - tar_meanY) * pair.weight;
        cross_covariance_matrix(1, 2) += (pair.src_point.y - src_meanY) * (pair.tar_point.z - tar_meanZ) * pair.weight;
        cross_covariance_matrix(2, 0) += (pair.src_point.z - src_meanZ) * (pair.tar_point.x - tar_meanX) * pair.weight;
        cross_covariance_matrix(2, 1) += (pair.src_point.z - src_meanZ) * (pair.tar_point.y - tar_meanY) * pair.weight;
        cross_covariance_matrix(2, 2) += (pair.src_point.z - src_meanZ) * (pair.tar_point.z - tar_meanZ) * pair.weight;
    }

    // compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // compute rotation
    Eigen::Matrix3d R = V * U.transpose();

    // compute translation
    Eigen::Vector3d t = Eigen::Vector3d(tar_meanX, tar_meanY, tar_meanZ) - R * Eigen::Vector3d(src_meanX, src_meanY, src_meanZ);

    // compute transformation
    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    this->current_transformation = transformation;
}
#endif