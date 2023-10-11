/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_PARAMETERS_H
#define SRC_PARAMETERS_H

#include <ros/ros.h>

namespace params {
    // icp mode enums
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
    
    // pipeline mode enums
    enum UpdateMode {
        PREVIOUS_FRAME,
        KEY_FRAME,
        MAP_MODE,
        };

    enum KeyFrameMode {
        TIME_MODE,
        OVERLAP_MODE,
        };
    
    
    enum RemoveMode {
        EUCLIDEAN_NORM,
        INF_NORM,
    };


    // icp parameters
    extern double max_distance; // should be > 0
    extern int max_iterations; // should be > 0
    extern double transformation_epsilon; // should be > 0
    extern WeightMode weight_mode; // choices: distance, uniform
    extern RejectMode reject_mode; // choices: none, threshold, percentage
    extern double reject_threshold; // should be > 0
    extern double reject_percentage; // should be in [0, 1]

    // pipeline parameters
    extern UpdateMode update_mode; // choices: previous_frame, key_frame, map_mode
    extern KeyFrameMode key_frame_mode; // choices: time_mode, overlap_mode
    extern double time_threshold; // only used for key_frame mode
    extern double overlap_threshold; // only used for key_frame mode
    extern int map_size;
    extern int map_range;
    extern RemoveMode remove_mode; // choices: euclidean_norm, inf_norm
    extern int map_sample_size; // add #map_sample_size many points to map in each iteration
   

    void readParameters(const ros::NodeHandle &nh);

    void readWeightMode(const ros::NodeHandle &nh);
    void readRejectMode(const ros::NodeHandle &nh);
    void readUpdateMode(const ros::NodeHandle &nh);
    void readKeyFrameMode(const ros::NodeHandle &nh);
    void readRemoveMode(const ros::NodeHandle &nh);
}

#endif //SRC_PARAMETERS_H
