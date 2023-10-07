#include "icp_point2plane.h"
#include "parameters.h"


pcl::PointCloud<pcl::PointNormal>::Ptr icp_point2plane::estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Code taken from https://pointclouds.org/documentation/tutorials/normal_estimation.html
    std::cout << "Entering estimate_normals..." << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    
    // Use all neighbors in a sphere of radius 3cm
    // TODO make this flexible
    //1m radius 
    ne.setRadiusSearch(5);
    // >30 neighbours
    //ne.setKSearch(5);

    // Compute the features
    ne.compute(*cloud_normals);
    for(int i = 0; i < cloud_normals->points.size(); i++){
        //std::cout <<"Normal: " << cloud_normals->points[i].normal_x << " " << cloud_normals->points[i].normal_y << " " << cloud_normals->points[i].normal_z << std::endl;
    }

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    std::cout << "Finishing estimate_normals..." << std::endl;
    return cloud_with_normals;
}    


void icp_point2plane::calculate_rotation() {
    // compute means of src and tar clouds
    std::cout << "Calculating rotation with p2lane" << std::endl;
    std::pair<pcl::PointXYZ, pcl::PointXYZ> means = calculate_means();
    double src_meanX = means.first.x;
    double src_meanY = means.first.y;
    double src_meanZ = means.first.z;
    double tar_meanX = means.second.x;
    double tar_meanY = means.second.y;
    double tar_meanZ = means.second.z;

    // Get number of points in the cloud
    int n = correspondence_pairs.size();
    // Now the goal is to construct A and b so that Ax = b can be solved via SVD
    
    //Eigen::matrixXd A(n,6);
    Eigen::Matrix<double, Eigen::Dynamic, 6> A(n,6);
    //Eigen::matrixXd b(n);
    Eigen::VectorXd b(n);

    int idx = 0;
    // Fill the b vector according to fromula in the paper
    // TODO check if we need to use the weight here
    // TODO check if we need to shift my mean
    for(const auto& pair: correspondence_pairs){
        b(idx) += pair.tar_point.normal_x * pair.tar_point.x;
        b(idx) += pair.tar_point.normal_y * pair.tar_point.y;
        b(idx) += pair.tar_point.normal_z * pair.tar_point.z;
        b(idx) -= pair.tar_point.normal_x * pair.src_point.x;
        b(idx) -= pair.tar_point.normal_y * pair.src_point.y;
        b(idx) -= pair.tar_point.normal_z * pair.src_point.z;
        idx++;
    }
    idx = 0;
    // Fill the A matrix according to formula in the paper
    for(const auto& pair: correspondence_pairs){
        // Entry 1-3 
        A(idx,0) = pair.tar_point.normal_z * pair.src_point.y - pair.tar_point.normal_y * pair.src_point.z;
        A(idx,1) = pair.tar_point.normal_x * pair.src_point.z - pair.tar_point.normal_z * pair.src_point.x;
        A(idx,2) = pair.tar_point.normal_y * pair.src_point.x - pair.tar_point.normal_x * pair.src_point.y;
        // Entry 4-6
        A(idx,3) = pair.tar_point.normal_x;
        A(idx,4) = pair.tar_point.normal_y;
        A(idx,5) = pair.tar_point.normal_z;
        idx++;
    }
    idx = 0;
    // Solution of the least squares problem is A^+ * b = x
    Eigen::MatrixXd A_inv = A.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double, 6, 1> x = A_inv * b;
    //Calculate rotation Matrix R (not the approximation!)
    Eigen::Matrix3d R;
    // First column
    R(0,0) = 1;
    R(1,0) = x(2);
    R(2,0) = -x(1);
    // Second column
    R(0,1) = x(0) * x(1) -x(2);
    R(1,1) = x(0) * x(1) * x(2) + 1;
    R(2,1) = x(0);
    // Third column
    R(0,2) = x(0) * x(2) + x(1);
    R(1,2) = x(1) * x(2) - x(0);
    R(2,2) = 1;

    Eigen::Matrix3d I = R * R.transpose();
    // compute transformation
    Eigen::Matrix4d transformation;
    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = R;
    // set the last column of transformation.block to the last three entries in x
    transformation.block<3, 1>(0, 3) = x.block<3, 1>(3, 0);

    std::cout << "Should be a transformation matrix: " << transformation << std::endl;
    current_transformation = transformation;
}


double icp_point2plane::calculate_error() {
    double error = 0.0;

    // define Affine3d transformation
    Eigen::Affine3d affine_transform;
    affine_transform.matrix() = current_transformation;

    for (const auto& pair: correspondence_pairs) {
        const pcl::PointXYZ transformed_point = pcl::transformPoint(pair.src_point, affine_transform);
        // calculate error as defined
        const double curr_error = (pair.tar_point.x - transformed_point.x) * pair.tar_point.normal_x 
            + (pair.tar_point.y - transformed_point.y) * pair.tar_point.normal_y 
            + (pair.tar_point.z - transformed_point.z) * pair.tar_point.normal_z;
        error += pair.weight * curr_error * curr_error;
        }

    return error;
}