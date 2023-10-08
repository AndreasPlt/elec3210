#include "icp_point2point.h"
#include "parameters.h"


void icp_point2point::calculate_rotation() {
    std::cout << "Calculating rotation with p2point" << std::endl;

    // compute means of src and tar clouds
    std::pair<pcl::PointXYZ, pcl::PointXYZ> means = calculate_means();
    double src_meanX = means.first.x;
    double src_meanY = means.first.y;
    double src_meanZ = means.first.z;
    double tar_meanX = means.second.x;
    double tar_meanY = means.second.y;
    double tar_meanZ = means.second.z;
    
    // compute cross covariance matrix
    Eigen::Matrix3d cross_covariance_matrix;
    cross_covariance_matrix.setZero();
    std::cout << "Init Cross covariance matrix: " << std::endl;
    std::cout << cross_covariance_matrix << std::endl;
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
    std::cout << "Final Cross covariance matrix: " << std::endl;
    std::cout << cross_covariance_matrix << std::endl;

    // compute SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Singular values: " << svd.singularValues() << std::endl;
    std::cout << "U matrix: " << svd.matrixU() << std::endl;
    std::cout << "V matrix: " << svd.matrixV() << std::endl;
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // compute rotation
    Eigen::Matrix3d R = V * U.transpose();
    std::cout << "Rotation matrix: " << std::endl;
    std::cout << R << std::endl;
    std::cout << "Determinant: " << R.determinant() << std::endl;

    // compute translation
    Eigen::Vector3d t = Eigen::Vector3d(tar_meanX, tar_meanY, tar_meanZ) - R * Eigen::Vector3d(src_meanX, src_meanY, src_meanZ);
    std::cout << "Translation vector: " << std::endl;
    std::cout << t << std::endl;

    // compute transformation
    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    current_transformation = transformation;
    std::cout << "-----" << std::endl;
}


double icp_point2point::calculate_error() {
    double error = 0.0;
    Eigen::Affine3d affine_transform;
    affine_transform.matrix() = current_transformation;

    for (const auto& pair: correspondence_pairs) {
        const pcl::PointXYZ transformed_point = pcl::transformPoint(pair.src_point, affine_transform);
        error += pair.weight * pcl::squaredEuclideanDistance(pair.tar_point, transformed_point);
    }

    return error;
}