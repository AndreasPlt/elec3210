#include "icp_point2plane.h"
#include "parameters.h"



// PointNormal: inline constexpr PointNormal (const _PointNormal &p) : PointNormal{p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature} {}

/**
 * @brief 
 * 
 * Code taken from https://pointclouds.org/documentation/tutorials/normal_estimation.html
 */
pcl::PointCloud<pcl::PointNormal>::Ptr icp_point2plane::estimate_normals() {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(tar_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr tar_cloud_normals(new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // Compute the features
    ne.compute(*tar_cloud_normals);

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*tar_cloud, *tar_cloud_normals, *tar_cloud_with_normals);

    return tar_cloud_with_normals;
}    


/**
 * @brief Calculates the rotation matrix and translation vector using point-to-plane distance for a single ICP iteration.
 * 
 * The function calculates the optimal rotation for the point cloud based on the point-to-plane approach 
 * introduced by Kok-Lim Low in 2004 (https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf).
 * The implementation is based on the linear approximation in the paper, solved with SVD.
 *
 */
void icp_point2plane::calculate_rotation() {
    // ist das relevant: https://github.com/symao/libicp/blob/master/src/icpPointToPlane.cpp ?

    // Get number of points in the cloud
    int n = correspondence_pairs.size();
    // Now the goal is to construct A and b so that Ax = b can be solved via SVD
    
    Eigen::matrixXf A(n,6);
    Eigen::matrixXf b(n);
    int idx = 0;
    // Fill the b vector according to fromula in the paper
    for(const auto& pair: correspondence_pairs){
        b(idx) += pair.tar_point.n_x * pair.tar_point.x;
        b(idx) += pair.tar_point.n_y * pair.tar_point.y;
        b(idx) += pair.tar_point.n_z * pair.tar_point.z;
        b(idx) -= pair.tar_point.n_x * pair.tar_point.x;
        b(idx) -= pair.tar_point.n_y * pair.src_point.y;
        b(idx) -= pair.tar_point.n_z * pair.src_point.z;
        idx++;
    }
    idx = 0;
    // Fill the A matrix according to formula in the paper
    for(const auto& pair: correspondence_pairs){
        // Entry 1-3 
        A(idx,0) = pair.tar_point.n_z * pair.src_point.y - pair.tar_point.n_y * pair.src_point.z;
        A(idx,1) = pair.tar_point.n_x * pair.src_point.z - pair.tar_point.n_z * pair.src_point.x;
        A(idx,2) = pair.tar_point.n_y * pair.src_point.x - pair.tar_point.n_x * pair.src_point.y;
        // Entry 4-6
        A(idx,3) = pair.tar_point.n_x;
        A(idx,4) = pair.tar_point.n_y;
        A(idx,5) = pair.tar_point.n_z;
        idx++;
    }
    idx = 0;
    // Solution of the least squares problem is A^+ * b = x
    Eigen::MatrixXd A_inv = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::x = A_inv.matmul(b);
    //Calculate rotation Matrix R (not the approximation!)
    Eigen::Matrix4d R;
    // First column
    R(0,0) = 1;
    R(1,0) = x(2);
    R(2,0) = -x(1);
    R(3,0) = 0;
    // Second column
    R(0,1) = x(0) * x(1) -x(2);
    R(1,1) = x(0) * x(1) * x(2) + 1;
    R(2,1) = x(0);
    R(3,1) = 0;
    // Third column
    R(0,2) = x(0) * x(2) + x(1);
    R(1,2) = x(1) * x(2) - x(0);
    R(2,2) = 1;
    R(3,2) = 0;
    // fourth column
    R(0,3) = 0;
    R(1,3) = 0;
    R(2,3) = 0;
    R(3,3) = 1;

    // compute translation
    Eigen::Vector3d t = Eigen::Vector3d(tar_meanX, tar_meanY, tar_meanZ) - R * Eigen::Vector3d(src_meanX, src_meanY, src_meanZ);

    // compute transformation
    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    this->current_transformation = transformation;
}


double icp_point2plane::calculate_error() {
    double error = 0.0;
    Eigen::Affine3d affine_transform;
    affine_transform.matrix() = current_transformation;
    for (const auto& pair: correspondence_pairs) {
        const pcl::PointXYZ transformed_point = pcl::transformPoint(pair.src_point, affine_transform);
        const double curr_error = (pair.tar_point.x - transformed_point.x) * pair.tar_point.normal_x 
            + (pair.tar_point.y - transformed_point.y) * pair.tar_point.normal_y 
            + (pair.tar_point.z - transformed_point.z) * pair.tar_point.normal_z;
        error += curr_error * curr_error;
    }
    return error;
}