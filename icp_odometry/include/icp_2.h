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

struct correspondence_pair_2 {
    pcl::PointXYZ src_point;
    pcl::PointXYZ tar_point;
    double distance;
    double weight;
    correspondence_pair_2(pcl::PointXYZ src, pcl::PointXYZ tar): weight(1), distance(0) {
        src_point = src;
        tar_point = tar;
    };
};

class icp_2 {
    public:
        icp_2();
        ~icp_2(){};

        void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr _src_cloud);
        void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud);

        void align(pcl::PointCloud<pcl::PointXYZ>::Ptr output, Eigen::Matrix4d init_guess);

        Eigen::Matrix4d getFinalTransformation();
        double getError();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_transformed;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> tar_tree;
        std::vector<correspondence_pair_2> correspondence_pairs;
        Eigen::Matrix4d final_transformation;

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