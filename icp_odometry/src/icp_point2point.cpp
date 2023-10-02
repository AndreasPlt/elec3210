#include "icp_point2point.h"
#include "parameters.h"

void icp_point2point::determine_corresponding_points() {

}

/**
 * @brief Calculates the rotation matrix and translation vector using point-to-point distance for a single ICP iteration.
 * 
 * This function first calculates the means of the source and target clouds. It then calculates the cross covariance matrix
 * and performs SVD to obtain the rotation matrix. The translation vector is then calculated using the means and rotation matrix.
 * The transformation is then applied to the source cloud.
 */
void icp_point2point::calculate_rotation() {

    // compute means of src and tar clouds
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
    
    // compute cross covariance matrix
    Eigen::Matrix3d cross_covariance_matrix;
    cross_covariance_matrix.setZero();
    for (const auto& pair: correspondence_pairs) {
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

/**
 * @brief Calculates the error of the current transformation.
 * 
 * This function calculates the error of the current transformation by summing the squared distances
 * between the source and target points.
 * 
 * @return double The error of the current transformation.
 */
double icp_implementation::calculate_error() {
    double error = 0.0;
    Eigen::Affine3d affine_transform;
    affine_transform.matrix() = current_transformation;
    for (const auto& pair: correspondence_pairs) {
        const pcl::PointXYZ transformed_point = pcl::transformPoint(pair.src_point, affine_transform);
        error += pcl::squaredEuclideanDistance(pair.tar_point, transformed_point);
    }
    return error;
}