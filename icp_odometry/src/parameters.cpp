/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#include "parameters.h"

namespace params{
    // icp parameters
    double max_distance = 1.0;
    int max_iterations = 100;
    int update_mode = 0;
    int key_frame_mode = 0;
    double time_threshold = 0.001;
    int map_size = 5000;
    int map_range = 5;
    int remove = 1;

    void readParameters(const ros::NodeHandle& nh){
        std::cout << "Reading parameters..." << std::endl;
        nh.getParam("/icp/max_iterations", max_iterations);
        std::cout << "max_iterations: " << max_iterations << std::endl;
        nh.getParam("/icp/max_distance", max_distance);
        std::cout << "max_distance: " << max_distance << std::endl;
        nh.getParam("/icp/update_mode", update_mode);
        std::cout << "update_mode: " << update_mode << std::endl;
        nh.getParam("/icp/key_frame_mode", key_frame_mode);
        std::cout << "key_frame_mode: " << key_frame_mode << std::endl;
        nh.getParam("/icp/time_threshold", time_threshold);
        std::cout << "time_threshold: " << time_threshold << std::endl;
        nh.getParam("/icp/map_size", map_size);
        std::cout << "map_size: " << map_size << std::endl;
        nh.getParam("/icp/map_range", map_range);
        std::cout << "map_range: " << map_range << std::endl;
        nh.getParam("/icp/remove", remove);
        std::cout << "remove: " << remove << std::endl;
    }
}
