/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "parameters.h"
#include <string.h>

namespace params{

    // icp parameters
    double max_distance = 100000;
    int max_iterations = 100;

    UpdateMode update_mode = PREVIOUS_FRAME;
    KeyFrameMode key_frame_mode = TIME_MODE;
    double time_threshold = 0.001;
    int map_size = 5000;
    int map_range = 5;
    RemoveMode remove_mode = INF_NORM;
    
    double transformation_epsilon = 1e-6;
    WeightMode weight_mode = UNIFORM;
    RejectMode reject_mode = THRESHOLD;
    double reject_threshold = 0.1;
    double reject_percentage = 0.1;
    bool debug_mode = false;

    void readParameters(const ros::NodeHandle& nh){
        std::cout << "Reading parameters..." << std::endl;
        
        // icp parameters
        nh.getParam("/icp/max_iterations", max_iterations);
        assert(max_iterations > 0);
        std::cout << "max_iterations: " << max_iterations << std::endl;
    
        nh.getParam("/icp/max_distance", max_distance);
        assert(max_distance > 0);
        std::cout << "max_distance: " << max_distance << std::endl;

        nh.getParam("/icp/transformation_epsilon", transformation_epsilon);
        assert(transformation_epsilon > 0);
        std::cout << "transformation_epsilon: " << transformation_epsilon << std::endl;

        nh.getParam("/icp/reject_threshold", reject_threshold);
        assert(reject_threshold > 0);
        std::cout << "reject_threshold: " << reject_threshold << std::endl;

        nh.getParam("/icp/reject_percentage", reject_percentage);
        assert(reject_percentage >= 0 && reject_percentage <= 1);
        std::cout << "reject_percentage: " << reject_percentage << std::endl;

        readWeightMode(nh);
        readRejectMode(nh);

        nh.getParam("/icp/debug_mode", debug_mode);
        std::cout << "debug_mode: " << debug_mode << std::endl;

        
        // pipeline parameters
        nh.getParam("/icp/map_size", map_size);
        // add assert
        std::cout << "map_size: " << map_size << std::endl;

        nh.getParam("/icp/map_range", map_range);
        // add assert
        std::cout << "map_range: " << map_range << std::endl;

    
        readUpdateMode(nh);
        readKeyFrameMode(nh);
        readRemoveMode(nh);

        // 
        std::cout << "Parameters reading finished." << std::endl;        
    }

    // icp mode parser
    void readWeightMode(const ros::NodeHandle& nh){
        // parsing weight_mode
        static std::unordered_map<std::string, WeightMode> const weight_mode_map = { 
            {"distance_inverse", DISTANCE_INVERSE}, 
            {"distance_max_scaling", DISTANCE_MAX_SCALING},
            {"uniform", UNIFORM}
        };

        std::string _weight_mode;

        nh.getParam("/icp/weight_mode", _weight_mode);
        auto weight_it = weight_mode_map.find(_weight_mode);
        if (weight_it == weight_mode_map.end()) {
            std::cout << "Invalid weight_mode. Using default (uniform)" << std::endl;
            weight_mode = UNIFORM;
        } else {
            weight_mode = weight_it->second;
            std::cout << "weight_mode: " << _weight_mode << std::endl;
        }
    }

    void readRejectMode(const ros::NodeHandle& nh){
        // parsing reject_mode
        static std::unordered_map<std::string, RejectMode> const reject_mode_map = { 
            {"none", NONE}, 
            {"threshold", THRESHOLD},
            {"percentage", PERCENTAGE},
        };

        std::string _reject_mode;

        nh.getParam("/icp/reject_mode", _reject_mode);
        auto reject_it = reject_mode_map.find(_reject_mode);
        if (reject_it == reject_mode_map.end()) {
            std::cout << "Invalid reject_mode. Using default (none)" << std::endl;
            reject_mode = NONE;
        } else {
            reject_mode = reject_it->second;
            std::cout << "reject_mode: " << _reject_mode << std::endl;
        }
    }

    // pipeline mode parser
    void readUpdateMode(const ros::NodeHandle& nh){
        // parsing update_mode
        static std::unordered_map<std::string, UpdateMode> const update_mode_map = { 
            {"previous_frame", PREVIOUS_FRAME}, 
            {"key_frame", KEY_FRAME},
            {"map_mode", MAP_MODE},
        };

        std::string _update_mode;

        nh.getParam("/icp/update_mode", _update_mode);
        auto update_it = update_mode_map.find(_update_mode);
        if (update_it == update_mode_map.end()) {
            std::cout << "Invalid update_mode. Using default (previous_frame)" << std::endl;
            update_mode = PREVIOUS_FRAME;
        } else {
            update_mode = update_it->second;
            std::cout << "update_mode: " << _update_mode << std::endl;
        }
    }

    void readKeyFrameMode(const ros::NodeHandle& nh){
        // parsing key_frame_mode
        static std::unordered_map<std::string, KeyFrameMode> const key_frame_mode_map = { 
            {"time_mode", TIME_MODE}, 
            {"overlap_mode", OVERLAP_MODE},
        };

        std::string _key_frame_mode;

        nh.getParam("/icp/key_frame_mode", _key_frame_mode);
        auto key_frame_it = key_frame_mode_map.find(_key_frame_mode);
        if (key_frame_it == key_frame_mode_map.end()) {
            std::cout << "Invalid key_frame_mode. Using default (time_mode)" << std::endl;
            key_frame_mode = TIME_MODE;
        } else {
            key_frame_mode = key_frame_it->second;
            std::cout << "key_frame_mode: " << _key_frame_mode << std::endl;
        }
    }

    void readRemoveMode(const ros::NodeHandle& nh){
        // parsing remove_mode
        static std::unordered_map<std::string, RemoveMode> const remove_mode_map = { 
            {"euclidean_norm", EUCLIDEAN_NORM}, 
            {"inf_norm", INF_NORM},
        };

        std::string _remove_mode;

        nh.getParam("/icp/remove_mode", _remove_mode);
        auto remove_it = remove_mode_map.find(_remove_mode);
        if (remove_it == remove_mode_map.end()) {
            std::cout << "Invalid remove_mode. Using default (euclidean_norm)" << std::endl;
            remove_mode = EUCLIDEAN_NORM;
        } else {
            remove_mode = remove_it->second;
            std::cout << "remove_mode: " << _remove_mode << std::endl;
        }
    }
}
