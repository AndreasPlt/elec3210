/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "parameters.h"

namespace params{

    // icp parameters
    double max_distance = 100000;
    int max_iterations = 100;
    double transformation_epsilon = 1e-6;
    std::string icp_mode = "point2point";
    WeightMode weight_mode = UNIFORM;
    RejectMode reject_mode = THRESHOLD;
    double reject_threshold = 0.1;
    double reject_percentage = 0.1;

    void readParameters(const ros::NodeHandle& nh){
        static std::unordered_map<std::string, WeightMode> const weight_mode_map = { 
            {"distance_inverse", DISTANCE_INVERSE}, 
            {"distance_max_scaling", DISTANCE_MAX_SCALING},
            {"uniform", UNIFORM}
            };
        static std::unordered_map<std::string, RejectMode> const reject_mode_map = { 
            {"none", NONE}, 
            {"threshold", THRESHOLD},
            {"percentage", PERCENTAGE},
            };
        

        std::string _weight_mode ;
        std::string _reject_mode;


        std::cout << "Reading parameters..." << std::endl;
        nh.getParam("/icp/max_iterations", max_iterations);
        std::cout << "max_iterations: " << max_iterations << std::endl;
        nh.getParam("/icp/max_distance", max_distance);
        std::cout << "max_distance: " << max_distance << std::endl;
        nh.getParam("/icp/icp_mode", icp_mode);
        std::cout << "icp_mode: " << icp_mode << std::endl;

        // parsing weight_mode
        nh.getParam("/icp/weight_mode", _weight_mode);
        auto weight_it = weight_mode_map.find(_weight_mode);
        if (weight_it == weight_mode_map.end()) {
            std::cout << "Invalid weight_mode. Using default (uniform)" << std::endl;
            weight_mode = UNIFORM;
        } else {
            weight_mode = weight_it->second;
            std::cout << "weight_mode: " << _weight_mode << std::endl;
        }
        
        // parsing reject_mode
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
}
