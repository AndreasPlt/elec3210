#ifndef ICP_IMPL_H
#define ICP_IMPL_H

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <unordered_set>
#include <string>
#include "parameters.h"

struct correspondence_pair {
    pcl::PointXYZ src_point;
    pcl::PointXYZ tar_point;
    double weight;
    double distance;
    correspondence_pair(): weight(1), distance(0) {};
};
class icp_implementation {
public:
    // constructors
    icp_implementation() {};
    ~icp_implementation() {};

    // getters and setters
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getInputSource() {
        return this->src_cloud;
    }
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getInputTarget() {
        return this->tar_cloud;
    }
    inline Eigen::Matrix4d getFinalTransformation() {
        return this->final_transformation;
    }
    inline void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) {
        this->src_cloud = src_cloud;
        this->src_cloud_transformed = src_cloud;
    }
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud) {
        this->tar_cloud = tar_cloud;
        this->tar_kd_tree.setInputCloud(tar_cloud);
    }
    
    // main functions
    void align(pcl::PointCloud<pcl::PointXYZ> &output_cloud, Eigen::Matrix4d init_guess);

private:
    // private attributes
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_transformed;
    pcl::KdTreeFLANN<pcl::PointXYZ> tar_kdtree;
    Eigen::Matrix4d current_transformation;
    Eigen::Matrix4d final_transformation;
    std::vector<correspondence_pair> correspondence_pairs;

    // private functions
    pcl::PointXYZ get_nearest_point(pcl::PointXYZ point);
    void determine_corresponding_points();
    void reject_pairs();
    void weight_pairs();
    void calculate_rotation_point2point();
    void calculate_rotation_point2plane();
    double calculate_error_point2point();
};

#endif