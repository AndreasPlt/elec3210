/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"
#include <pcl/registration/icp.h>
#include "parameters.h"



Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
	//return Eigen::Matrix4d::Identity();
    /*if(params::icp_mode == "point2plane"){
        return point_to_plane_icp(src_cloud, tar_cloud, init_guess);
    }*/
    //else 
    if(params::icp_mode == "point2point"){
        return point_to_point_icp(src_cloud, tar_cloud, init_guess);
    }
    
    std::cout << "Please select correct mode for association." << std::endl;
    exit(1);
}

Eigen::Matrix4d point_to_plane_icp2(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    icp_point2plane icp;
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tar_cloud);
    icp.align(init_guess);
    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}

Eigen::Matrix4d point_to_plane_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;

    icp_point2plane icp_2;

    pcl::PointCloud<pcl::PointNormal>::Ptr src_cloud_normals = icp_2.estimate_normals(src_cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_cloud_normals = icp_2.estimate_normals(tar_cloud);

    icp.setInputSource(src_cloud_normals);
    icp.setInputTarget(tar_cloud_normals);
    icp.setMaximumIterations(params::max_iterations);  // set maximum iteration
    icp.setTransformationEpsilon(1e-6);  // set transformation epsilon
    icp.setMaxCorrespondenceDistance(params::max_distance);  // set maximum correspondence distance
    std::cout << "Start aligning" << std::endl;
    pcl::PointCloud<pcl::PointNormal> aligned_cloud;
    icp.align(aligned_cloud, init_guess.cast<float>());

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}

Eigen::Matrix4d point_to_point_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    icp_point2point icp;
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tar_cloud);
    icp.align(init_guess);
    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}

Eigen::Matrix4d icp_registration2(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    // This is an example of using pcl::IterativeClosestPoint to align two point clouds
    // In your project, you should implement your own ICP algorithm!!!
    // In your implementation, you can use KDTree in PCL library to find nearest neighbors
    // Use chatGPT, google and github to learn how to use PCL library and implement ICP. But do not copy totally. TA will check your code using advanced tools.
    // If you use other's code, you should add a reference in your report. https://registry.hkust.edu.hk/resource-library/academic-integrity

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tar_cloud);
    icp.setMaximumIterations(params::max_iterations);  // set maximum iteration
    icp.setTransformationEpsilon(1e-6);  // set transformation epsilon
    icp.setMaxCorrespondenceDistance(params::max_distance);  // set maximum correspondence distance
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, init_guess.cast<float>());

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}

