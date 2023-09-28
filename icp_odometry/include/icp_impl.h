#ifndef ICP_IMPL_H
#define ICP_IMPL_H

struct correspondence_pair {
    pcl::PointXYZ src_point;
    pcl::PointXYZ tar_point;
    double weight;
    double distance;
    bool rejected;
    correspondence_pair(): weight(0), distance(0), rejected(false) {};
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
    inline int getMaxIterations() {
        return this->max_iterations;
    }
    inline double getTransformationEpsilon() {
        return this->transformation_epsilon;
    }
    inline double getMaxCorrespondenceDistance() {
        return this->max_correspondence_distance;
    }
    inline void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) {
        this->src_cloud = src_cloud;
        this->src_cloud_transformed = src_cloud;
    }
    inline void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud) {
        this->tar_cloud = tar_cloud;
        this->tar_kd_tree.setInputCloud(tar_cloud);
    }
    inline void setMaxIterations(int max_iterations) {
        this->max_iterations = max_iterations;
    }
    inline void setTransformationEpsilon(double transformation_epsilon) {
        this->transformation_epsilon = transformation_epsilon;
    }
    inline void setMaxCorrespondenceDistance(double max_distance) {
        this->max_distance = max_distance;
    }
    inline Eigen::Matrix4d getFinalTransformation() {
        return this->final_transformation;
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
    int max_iterations;
    double transformation_epsilon;
    double max_correspondence_distance;

    // private functions
    pcl::PointXYZ get_nearest_point(pcl::PointXYZ point);
    void determine_corresponding_points();
    void reject_pairs();
    void weight_pairs();
    void calculate_rotation_point2point();
    void calculate_rotation_point2plane();
    double calculate_error();
};

#endif