#ifndef ICP_POINT2PLANE_H
#define ICP_POINT2PLANE_H

#include "icp_general.h"

class icp_point2plane: public icp_general<pcl::PointXYZ, pcl::PointNormal> {
public:
    // constructors
    icp_point2plane();
    ~icp_point2plane(){};

    // main functions
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud) override {
        this->tar_cloud = estimate_normals(tar_cloud);
        this->tar_kdtree.setInputCloud(this->tar_cloud);
    }

private:
    // private functions
    pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void calculate_rotation() override;
    double calculate_error() override;
};

#endif