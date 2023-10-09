/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_ICP_H
#define SRC_ICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "icp_general.h"
#include "icp_point2plane.h"
#include "icp_point2point.h"

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess);
Eigen::Matrix4d icp_registration2(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess);

Eigen::Matrix4d point_to_plane_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess);
Eigen::Matrix4d point_to_point_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess);


#endif //SRC_ICP_H
