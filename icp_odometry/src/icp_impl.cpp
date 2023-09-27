#ifndef ICP_IMPL_H
#define ICP_IMPL_H

#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include "parameters.h"
#include "icp_impl.h"


// icp_implementation constructor
icp_implementation::icp_implementation() {}

determine_corresponding_points(pcl::PointCloud<pcl::PointXYZ> &src_cloud, pcl::PointCloud<pcl::PointXYZ> &tar_cloud) {
    // build kd-tree for target cloud
    tar_kdtree.setInputCloud(tar_cloud);
    // for each point in source cloud
    for (int i = 0; i < src_cloud->points.size(); i++) {
        // find nearest point in target cloud
        pcl::PointXYZ nearest_point = getNearestPoint(src_cloud->points[i]);
        // add pair to corresponding points
        correspondence_pairs.push_back(std::make_pair(src_cloud->points[i], nearest_point));
    }
}


icp_implementation::getNearestPoint(pcl::PointXYZ point) {
        pcl::PointXYZ nearest_point;
        std::vector<int> pointIdxKNNSearch(1);
        std::vector<float> pointKNNSquaredDistance(1);
        tar_kdtree.nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        nearest_point = tar_cloud->points[pointIdxNKNSearch[0]];

        // TODO: multiple nearest points
        // radius search
        // check consistency
        return nearest_point;
    }
/**
 * \brief Assigns weights to correspondence pairs based on their distance.
 *
 * This function calculates the distance for each correspondence pair and sorts them
 * based on distance. It then assigns weights to pairs, where the first num_pairs will
 * have a weight of 1, and the rest will have a weight of 0.
 *
 * \param percentage The percentage of pairs to assign weight 1 (range: 0.0 to 1.0).
 * \return None.
 */
void icp_implementation::weight_pairs(float percentage) {
    //calculate the distance of each pair
    for (int i = 0; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].distance = calculate_distance(correspondence_pairs[i]);
    }
    //sort the pairs by distance
    std::sort(correspondence_pairs.begin(), correspondence_pairs.end(), compare_distance);
    // set the weight of the first num_pairs to 1, rest to zero
    int num_pairs = correspondence_pairs.size() * (1 - percentage);
    for (int i = 0; i < num_pairs; i++) {
        correspondence_pairs[i].weight = 1;
    }
    for (int i = num_pairs; i < correspondence_pairs.size(); i++) {
        correspondence_pairs[i].weight = 0;
    }
}

compare_distance(correspondence_pair pair1, correspondence_pair pair2) {
    return pair1.distance < pair2.distance;
}

icp_implementation::align(pcl::PointCloud<pcl::PointXYZ> &output_cloud, Eigen::Matrix4d init_guess) {
    // determine corresponding points
    determine_corresponding_points();
    // weight/reject pairs
    weight_pairs();
    reject_pairs();
    // compute translation and rotation
    Eigen::Matrix4d R = calculate_rotation();
    // apply R and t to all points
    
    pcl::transformPointCloud(*src_cloud, output_cloud, R);
    // compute error
    // check convergence (error > threshold)
        // if not converged, repeat
    // output final alignment
}
#endif