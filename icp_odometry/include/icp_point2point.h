#ifndef ICP_POINT2POINT_H
#define ICP_POINT2POINT_H

#include "icp_general.h"

class icp_point2point: public icp_general<pcl::PointXYZ, pcl::PointXYZ> {
public:
    // constructors
    /**
     * @brief Construct a new icp point2point object
     * 
     * This constructor does nothing.
     */
    icp_point2point(){};
    /**
     * @brief Destroy the icp point2point object
     * 
     * This destructor does nothing.
     */
    ~icp_point2point(){};

    /**
     * @brief Set the Input Target object
     * 
     * @param _tar_cloud 
     */
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) override {
        tar_cloud = _tar_cloud;
        tar_kdtree.setInputCloud(tar_cloud);
    }
    
    /**
     * @brief Set the Input Source object
     * 
     * @param _src_cloud 
     */
    inline void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud) override {
        src_cloud = _src_cloud;
        //pcl::copyPointCloud<pcl::PointXYZ>(*src_cloud, *src_cloud_transformed);
        src_cloud_transformed = _src_cloud;
    }

private:
    // private functions
    /**
    * @brief Calculates the rotation matrix and translation vector using the direct SVD-based method for a single ICP iteration.
    */
    void calculate_rotation() override;

    /**
    * @brief Calculates the error of the current transformation.
    * 
    * This function calculates the error of the current transformation by summing the weighted squared distances
    * between the source and target points.
    * 
    * @return double The error of the current transformation.
    */
    double calculate_error() override;
};
#endif