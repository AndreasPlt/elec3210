#ifndef ICP_IMPL_H
#define ICP_IMPL_H

class icp_implementation {
public:
    icp_implementation(void) {};
    ~icp_implementation(void) {};



    pcl::PointCloud<pcl::PointXYZ>::Ptr getInputSource() {
        return this->src_cloud;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getInputTarget() {
        return this->tar_cloud;
    }
    int getMaxIterations() {
        return this->max_iterations;
    }
    double getTransformationEpsilon() {
        return this->transformation_epsilon;
    }
    double getMaxCorrespondenceDistance() {
        return this->max_correspondence_distance;
    }
    void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) {
        this->src_cloud = src_cloud;
    }
    void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud) {
        this->tar_cloud = tar_cloud;
        this->tar_kd_tree.setInputCloud(tar_cloud);
    }
    void setMaxIterations(int max_iterations) {
        this->max_iterations = max_iterations;
    }
    void setTransformationEpsilon(double transformation_epsilon) {
        this->transformation_epsilon = transformation_epsilon;
    }
    void setMaxCorrespondenceDistance(double max_distance) {
        this->max_distance = max_distance;
    }
    
    void align(pcl::PointCloud<pcl::PointXYZ> &output_cloud, Eigen::Matrix4d init_guess);
    Eigen::Matrix4d getFinalTransformation();

    

    

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> tar_kdtree;
    
    int max_iterations;
    double transformation_epsilon;
    double max_correspondence_distance;

    pcl::PointXYZ getNearestPoint(pcl::PointXYZ point);
};

#endif