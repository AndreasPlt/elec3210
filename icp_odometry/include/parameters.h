/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H

#include <ros/ros.h>
#include <string>

namespace params {
    extern double max_distance;
    extern int max_iterations;
    extern double transformation_epsilon;
    extern std::string icp_mode; // choices: point2point, point2plane
    extern std::string weight_mode; // choices: distance, uniform
    extern std::string reject_mode; // choices: none, threshold, percentage
    double reject_threshold; // should be > 0
    double reject_percentage; // should be in [0, 1]

    void readParameters(const ros::NodeHandle &nh);
}

#endif //SRC_PARAMETERS_H
