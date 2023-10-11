#ifndef ICP_2_H
#define ICP_2_H

#include <Eigen/Core>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <limits>

/**
 * @brief Struct to store a pair of corresponding points, together with their distance and weight
 * 
 */
struct correspondence_pair_2 {
    pcl::PointXYZ src_point;
    pcl::PointXYZ tar_point;
    double distance;
    double weight;
    /**
     * @brief Construct a new correspondence pair 2 object
     * 
     * Weight is initialized to 1, distance is initialized to 0
     * 
     * @param src source point
     * @param tar target point
     */
    correspondence_pair_2(pcl::PointXYZ src, pcl::PointXYZ tar): weight(1), distance(0) {
        src_point = src;
        tar_point = tar;
    };
};

/**
 * @brief Class to perform ICP
 * 
 */
class icp_2 {
    public:

        /**
         * @brief Construct a new icp 2 object and initialize variables
         * 
         */
        icp_2();
        /**
         * @brief Destroy the icp 2 object.
         * 
         * Does nothing
         */
        ~icp_2(){};

        /**
         * @brief Set the Input Source object
         * 
         * @param _src_cloud 
         */
        void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud);
        /**
         * @brief Set the Input Target object
         * 
         * @param _tar_cloud 
         */
        void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud);

        /**
         * @brief Perform alignment between the source and target point clouds
         * 
         * @param output Stores the aligned source point cloud
         * @param init_guess Initial guess for the transformation. Defaults to identity
         */
        void align(pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity());

        /**
         * @brief Get the final rigid transformation matrix to align the source point cloud to the target point cloud
         * 
         * @return Eigen::Matrix4d 
         */
        Eigen::Matrix4d getFinalTransformation();
        /**
         * @brief Get the error of the alignment
         * 
         * @return double 
         */
        double getError();

    private:
        // private attributes
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_transformed;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> tar_tree;
        std::vector<correspondence_pair_2> correspondence_pairs;
        Eigen::Matrix4d final_transformation;

        // private functions
        void determine_corresponding_points();

        void weight_pairs();
        void weight_pairs_max_dist_scaling();
        void weight_pairs_distance_inverse();

        void reject_pairs();
        void reject_pairs_threshold();
        void reject_pairs_percentage();

        Eigen::Matrix4d compute_transformation();
        double _get_error(Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity());

};
#endif