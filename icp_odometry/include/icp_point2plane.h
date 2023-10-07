#ifndef ICP_POINT2PLANE_H
#define ICP_POINT2PLANE_H

#include "icp_general.h"

class icp_point2plane: public icp_general<pcl::PointXYZ, pcl::PointNormal> {
public:
    // constructors
    icp_point2plane() {};
    ~icp_point2plane(){};

    // main functions
    /**
     * @brief Set the Input Target object
     * 
     * This function estimates normals for the target cloud and sets the result as the internal target cloud.
     * The cloud should contain points of type pcl::PointXYZ.
     * 
     * @param _tar_cloud 
     */
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) override {
        tar_cloud = estimate_normals(_tar_cloud);
        std::cout << "start tree stuff" << std::endl;
        tar_kdtree.setInputCloud(tar_cloud);
        std::cout << "finished tree stuff" << std::endl;
    }
    
    /**
     * @brief Set the Input Source object. The cloud should contain points of type pcl::PointXYZ.
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
     * @brief Estimates normals for a point cloud using pcl::NormalEstimation.
     * 
     * @param cloud Point cloud with PointXYZ points to estimate normals for.
     * @return pcl::PointCloud<pcl::PointNormal>::Ptr Point cloud with PointNormal points, including the estimated normals.
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    /**
    * @brief Calculates the rotation matrix and translation vector using point-to-plane distance for a single ICP iteration.
    * 
    * The function calculates the optimal rotation for the point cloud based on the point-to-plane approach 
    * introduced by Kok-Lim Low in 2004 (https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf).
    * The implementation is based on the linear approximation in the paper, solved with SVD.
    */
    void calculate_rotation() override;
    
    /**
     * @brief Calculates the current error using the point-to-plane distance metric.
     * 
     * The error is calculated as follows:
     * 
     * \f$ \sum_{i=1}^{n} p_i (n_{i}^{T} (s_{i} - t_{i}))^{2} \f$
     * 
     * where \f$ n_{i} \f$ is the normal of the target point \f$ t_{i} \f$, \f$ s_{i} \f$ is the corresponding source point
     * and \f$ p_{i} \f$ is the weight of the correspondence pair.
     * @return double 
     */
    double calculate_error() override;
};

#endif