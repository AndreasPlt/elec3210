#ifndef ICP_POINT2PLANE_H
#define ICP_POINT2PLANE_H

#include "icp_general.h"

class icp_point2plane: public icp_general<pcl::PointXYZ, pcl::PointNormal> {
public:
    // constructors
    icp_point2plane() {};
    ~icp_point2plane(){};

    // main functions
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) override {
        tar_cloud = estimate_normals(_tar_cloud);
        std::cout << "start tree stuff" << std::endl;
        tar_kdtree.setInputCloud(tar_cloud);
        std::cout << "finished tree stuff" << std::endl;
    }
    
    inline void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud) override {
        src_cloud = _src_cloud;
        //pcl::copyPointCloud<pcl::PointXYZ>(*src_cloud, *src_cloud_transformed);
        src_cloud_transformed = _src_cloud;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
    // private functions
    
    void calculate_rotation() override;
    double calculate_error() override;
};

#endif