/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H

#include <ros/ros.h>

namespace params {
    enum WeightMode {
        DISTANCE_INVERSE,
        DISTANCE_MAX_SCALING, 
        UNIFORM,
        };

    enum RejectMode {
        NONE,
        THRESHOLD,
        PERCENTAGE,
        };

    extern double max_distance;
    extern int max_iterations;
    extern int update_mode; // 0: previous frame, 1: key frame, 2:map mode
    extern int key_frame_mode; // 0: time mode, 1: overlap mode
    extern double time_threshold; // only used for key frame mode
    extern int map_size;
    extern int map_range;
    extern int remove;
    extern double transformation_epsilon;
    extern std::string icp_mode; // choices: point2point, point2plane
    extern WeightMode weight_mode; // choices: distance, uniform
    extern RejectMode reject_mode; // choices: none, threshold, percentage
    extern double reject_threshold; // should be > 0
    extern double reject_percentage; // should be in [0, 1]

    void readParameters(const ros::NodeHandle &nh);
}

#endif //SRC_PARAMETERS_H
