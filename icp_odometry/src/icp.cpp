/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/
#include "icp.h"
#include <pcl/registration/icp.h>
#include "parameters.h"
#include <time.h>

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
    icp_2 icp;
    icp.setInputSource(src_cloud);
    icp.setInputTarget(tar_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(aligned_cloud, init_guess);

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}

Eigen::Matrix4d icp_registration_pcl(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {
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
    icp.setMaxCorrespondenceDistance(params::reject_threshold);  // set maximum correspondence distance
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, init_guess.cast<float>());

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    return transformation;
}