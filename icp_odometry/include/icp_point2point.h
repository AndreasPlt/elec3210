#include "icp_general.h"

class icp_point2point: public icp_general<pcl::PointXYZ, pcl::PointXYZ> {
public:
    // constructors
    icp_point2point(){};
    ~icp_point2point(){};

    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) override {
        tar_cloud = _tar_cloud;
        tar_kdtree.setInputCloud(tar_cloud);
    }
    inline void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud) override {
        src_cloud = _src_cloud;
        //pcl::copyPointCloud<pcl::PointXYZ>(*src_cloud, *src_cloud_transformed);
        src_cloud_transformed = _src_cloud;
    }

private:
    // private functions
    void calculate_rotation() override;
    double calculate_error() override;
};
