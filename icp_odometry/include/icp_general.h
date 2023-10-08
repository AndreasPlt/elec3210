#ifndef ICP_IMPL_H
#define ICP_IMPL_H

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
 * @brief Struct to store a pair of corresponding points, together with their weight and distance.
 * 
 * Weight is initialized to 1, and distance is initialized to 0.
 * 
 * @tparam S Point type of the source cloud.
 * @tparam T Point type of the target cloud.
 */
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
 * @brief General class template for ICP.
 * 
 * This class template is used to perform ICP on two point clouds. It is a general class template,
 * and should not be used directly. Instead, use one of the derived classes.
 * 
 * @tparam S internal point type of source cloud. Should have x, y, z attributes.
 * @tparam T internal point type of target cloud. Should have a constructor that takes in x, y, z as arguments.
 */
template <class S, class T>
class icp_general {
public:
    // constructors
    /**
     * @brief Construct a new icp_general object and initialize the attributed correctly.
     * 
     * This constructor initializes the object's attributes and validates the ICP parameters.
     * 
     */
    icp_general();
    /**
     * @brief Destroy the icp general object
     * 
     * This destructor does nothing.
     */
    ~icp_general(){};

    /**
     * @brief Get the Final Transformation object
     * 
     * @return Eigen::Matrix4d The final transformation matrix.
     */
    inline Eigen::Matrix4d getFinalTransformation() {
        return this->final_transformation;
    }
    /**
     * @brief Set the Input Source object. The cloud should contain points of type pcl::PointXYZ.
     * 
     * @param src_cloud 
     */
    virtual void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud) = 0; 
    /**
     * @brief Set the Input Target object. The cloud should contain points of type pcl::PointXYZ.
     * 
     * @param _tar_cloud 
     */
    virtual void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr _tar_cloud) = 0;
    
    // main functions
    /**
     * @brief Performs alignment of the source cloud to the target cloud using ICP.
     * 
     * This function performs alignment of the source cloud to the target cloud using ICP. 
     * The result is stored in the final_transformation attribute.
     * 
     * @param init_guess Initial guess for the transformation matrix. Defaults to identity matrix.
     */
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
    /**
     * @brief Calculates the means of the source and target clouds.
     * 
     * @return std::pair<pcl::PointXYZ, pcl::PointXYZ> Pair of the means of the source and target clouds.
     */
    std::pair<pcl::PointXYZ, pcl::PointXYZ> calculate_means();
    /**
     * @brief Calculates the transformation for a single ICP step.
     * 
     * The transformation is stored in the current_transformation attribute. Note that this function
     * calculates the transformation between the currently transformed source cloud and the target cloud.
     */
    virtual void calculate_rotation() = 0;

    /**
     * @brief Calculates the error between the source and target clouds, using the specified error metric.
     * 
     * @return double The error between the source and target clouds.
     */
    virtual double calculate_error() = 0;

private:
    // private functions
    /**
    * @brief Obtain the nearest point in the target cloud to a given point.
    * 
    * This function uses a KDTree to find the nearest point in the target cloud to a given point.
    * @param point Point to find nearest point to.
    * @return pcl::PointXYZ Nearest point in target cloud.
    */
    T get_nearest_point(S point);

    /**
    * @brief Determines the corresponding points between the source and target clouds.
    * 
    * This function uses a KDTree to find the nearest point in the target cloud to each point in the source cloud.
    * It then stores the corresponding points in a vector of correspondence pairs (correspondence_pairs).
    * 
    * @return None.
    */
    void determine_corresponding_points();

    /**
     * @brief Rejects pairs based on a top percentage of pairs with the largest distance.
     * 
     * This function sorts the correspondence pairs by distance, and rejects the top percentage of pairs.
     * The percentage is specified in the params.h file. Note that the worst (1-p)% of pairs are rejected.
     */
    void reject_pairs_trimming();
    /**
     * @brief Rejects pairs based on a threshold distance.
     * 
     * This function rejects pairs with a distance greater than the threshold distance. The threshold distance
     * is specified in the params.h file.
     */
    void reject_pairs_threshold();
    /**
     * @brief Weights pairs based on their distance.
     * 
     * This function weights each pair based on their distance. The weight is calculated as follows:
     * 
     * \f$ w = 1 - \frac{d^2}{d'^*} \f$
     * 
     * Where \f$ d \f$ is the distance between the pair, and \f$ d'^* \f$ is the maximum distance between
     * any pair.
     */
    void weight_pairs_distance();
};

#endif