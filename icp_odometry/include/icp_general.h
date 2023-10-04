#ifndef ICP_IMPL_H
#define ICP_IMPL_H

#include <Eigen/Core>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/eigen.h>
#include <pcl/features/normal_3d.h>
#include <limits>


template <class S, class T>
struct correspondence_pair {
    S src_point;
    T tar_point;
    double weight;
    double distance;
    correspondence_pair(S src, T tar): weight(1), distance(0) {
        src_point = src;
        tar_point = tar;
    };
};

/**
 * @brief 
 * 
 * @tparam S internal point type of source cloud
 * @tparam T internal point type of target cloud
 */
template <class S, class T>
class icp_general {
public:
    // constructors
    icp_general();
    ~icp_general(){};

    // getters and setters
    /**
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getInputSource() {
        return this->src_cloud;
    }
    inline pcl::PointCloud<pcl::PointXYZ>::Ptr getInputTarget() {
        return this->tar_cloud;
    }
    */
    inline Eigen::Matrix4d getFinalTransformation() {
        return this->final_transformation;
    }
    virtual void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) = 0; 
    virtual void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) = 0;
    
    // main functions
    void align(Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity());

protected:
    // private attributes
    typename pcl::PointCloud<S>::Ptr src_cloud;
    typename pcl::PointCloud<T>::Ptr tar_cloud;
    typename pcl::PointCloud<S>::Ptr src_cloud_transformed;
    pcl::KdTreeFLANN<T> tar_kdtree;
    Eigen::Matrix4d current_transformation;
    Eigen::Matrix4d final_transformation;
    typename std::vector<correspondence_pair<S, T>> correspondence_pairs;

    // virtual functions
    std::pair<pcl::PointXYZ, pcl::PointXYZ> calculate_means();
    virtual void calculate_rotation() = 0;
    virtual double calculate_error() = 0;

private:
    // private functions
    T get_nearest_point(S point);
    void determine_corresponding_points();
    void reject_pairs_trimming();
    void reject_pairs_threshold();
    void weight_pairs_distance();
};

#endif