#include "odometry.h"
#include "parameters.h"
#include "global_definition.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <math.h>
#include <time.h>
#include <pcl/filters/extract_indices.h>
#include <algorithm>
#include <string.h>
#include <vector>
#include <float.h>



using namespace std;
using namespace Eigen;


OdomICP::~OdomICP() {}

OdomICP::OdomICP(ros::NodeHandle &nh):
        nh_(nh) {
//    initialize variables here
    Twb = Eigen::Matrix4d::Identity(); // initial pose
    Twb_prev = Eigen::Matrix4d::Identity();
    deltaT_pred = Eigen::Matrix4d::Identity();
    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>);
    refCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mapCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

//    initialze downsample filter here
    const double scan_leaf_size = 0.5, map_leaf_size = 0.1;
    dsFilterScan.setLeafSize(scan_leaf_size, scan_leaf_size, scan_leaf_size);
    dsFilterMap.setLeafSize(map_leaf_size, map_leaf_size, map_leaf_size);

//    initialize ros publisher
    lidar_sub = nh_.subscribe("/velodyne_points", 1, &OdomICP::cloudHandler, this);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("icp_odometry", 1);
    path_pub = nh_.advertise<nav_msgs::Path>("icp_path", 1);
    scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("current_scan", 1);
    map_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);

//    traj_file.open(WORK_SPACE_PATH + "/../dataset/true_trajectory.txt");
    // convert WORK_SPACE_PATH + "/../dataset/true_trajectory.txt" to const char
    std::string s = WORK_SPACE_PATH + "/../dataset/est_trajectory.txt"; 
    const char* filename = s.c_str(); 
    std::ofstream ofs;
    ofs.open(filename, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
}

void OdomICP::run() {
    auto started = std::chrono::high_resolution_clock::now();
    ros::Rate rate(1000);
    while (ros::ok()){
        if (cloudQueue.empty()){
            rate.sleep();
            continue;
        }

        cloudQueueMutex.lock();
        cloudHeader = cloudQueue.front().first;
        laserCloudIn = parseCloud(cloudQueue.front().second);
        cloudQueue.pop();
        cloudQueueMutex.unlock();

        if (firstFrame){
            firstFrame = false;
            Twb = Eigen::Matrix4d::Identity(); // initial pose
            Twb_prev = Eigen::Matrix4d::Identity();
            deltaT_pred = Eigen::Matrix4d::Identity();
            *refCloud = *laserCloudIn;
            //store current time (For keyframe time mode)
            last_update_time = std::clock();
            continue;
        }

        timer.tic();
        // Odometry estimation

        // 1. preprocess: downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        //Filter Laser Scan cloud
        dsFilterScan.setInputCloud(laserCloudIn);
        dsFilterScan.filter(*laserCloud_filtered);
        //Filter Map Cloud
        dsFilterMap.setInputCloud(refCloud);
        dsFilterMap.filter(*refCloud_filtered);

        // 2. icp
        // Create initial guess based on previous pose
        // invert Twb_prev 
        Eigen::Matrix4d Twb_prev_inv = Eigen::Matrix4d::Identity();
        Twb_prev_inv.block<3,3>(0,0) = Twb.block<3,3>(0,0).transpose();
        Twb_prev_inv.block<3,1>(0,3) = -Twb.block<3,3>(0,0).transpose() * Twb.block<3,1>(0,3);
        // Get T_{w->c} with old laserCloud, initial guess is the last T_{p->c} transformed with the last deltaT_pred (assuming we we have the same motion as last time)
        Twb = icp_registration(laserCloud_filtered, refCloud_filtered, deltaT_pred * Twb);
        // Calucalte delta for next initial guess (based on constant motion model)
        deltaT_pred = Twb * Twb_prev_inv;
        // extract the rotation on the z-axis 
        double phi = std::atan2(Twb(1,0),Twb(0,0));
        Eigen::Matrix3d euler_z = Eigen::Matrix3d::Identity();
        euler_z(0,0) = std::cos(phi);
        euler_z(1,0) = std::sin(phi);
        euler_z(0,1) = -std::sin(phi);
        euler_z(1,1) = std::cos(phi);
        Twb.block<3,3>(0,0) = euler_z;
        // no movement on the z-axis
        Twb(2,3) = 0;

        // transform the scan and store in ref cloud for publishing
        switch(params::update_mode){
            //previous frame mode
            case params::PREVIOUS_FRAME: 
                pcl::transformPointCloud(*laserCloudIn, *refCloud, Twb.cast<float>());
                break;

            //key frame mode
            case params::KEY_FRAME: 
                switch(params::key_frame_mode) {
                    case params::TIME_MODE:
                        if( (std::clock() - last_update_time) / CLOCKS_PER_SEC > params::time_threshold){
                            //update keyframe
                            pcl::transformPointCloud(*laserCloudIn, *refCloud, Twb.cast<float>());
                            //update time stamp
                            last_update_time = std::clock();
                        }   
                        break;

                    case params::OVERLAP_MODE:
                        //calculate bounding box intersection for both point clouds
                        if ( calc_intersection() < params::overlap_threshold){
                            //update keyframe
                            pcl::transformPointCloud(*laserCloudIn, *refCloud, Twb.cast<float>());
                            std::cout << "updated refCloud" << std::endl;
                        }
                        break;
                }
                break;

            //map mode
            case params::MAP_MODE:
                if( (double)(std::clock() - last_update_time) / (double)CLOCKS_PER_SEC > params::time_threshold){
                    //update keyframe
                    pcl::transformPointCloud(*laserCloudIn, *refCloud, Twb.cast<float>());
                    //update time stamp
                    last_update_time = std::clock();
                    // Transform current scan
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::transformPointCloud(*laserCloudIn, *transformed_scan, Twb.cast<float>());
                    // Extend map
                    *refCloud += *transformed_scan;
                    // remove points that are too far away from current position
                    switch(params::remove_mode){
                        // remove points based on 2-norm
                        case params::EUCLIDEAN_NORM:
                            remove_euclidean();
                            break;
                        case params::INF_NORM:
                        // remove points based on inf-norm
                            remove_inf();
                            break;
                    }
                    // downsample the map to a given map size
                    pcl::RandomSample<pcl::PointXYZ> random_sample;
                    random_sample.setInputCloud(refCloud);
                    random_sample.setSample(params::map_size);
                    random_sample.filter(*refCloud);
                    break;
                }
        }

        //store the scan in the map cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*laserCloudIn, *transformed_scan, Twb.cast<float>());
        //downsample the scan to be memory efficient (100 points per scan)
        pcl::RandomSample<pcl::PointXYZ> random_sample;
        random_sample.setInputCloud(transformed_scan);
        random_sample.setSample(params::map_sample_size);
        random_sample.filter(*transformed_scan);
        //add the scan to the map
        *mapCloud += *transformed_scan;

        //tic toc lol
        timer.toc();
        //4.5 store trajectory
        std::vector<Pose> est;
        est.push_back({cloudHeader.stamp.toSec(), Twb});
        store_data(est);
        // 5. publish result
        publishResult();
        rate.sleep();
    }
    auto done = std::chrono::high_resolution_clock::now();
    std::cout << "ICP took the following time: " << std::chrono::duration_cast<std::chrono::seconds>(done-started).count() << "s" << std::endl;
}
/**
 * @brief Store a vector of poses in a text file.
 *
 * This function appends the provided poses and their timestamps to a text file
 * located at the specified path.
 *
 * @param data A vector of Pose objects to be stored.
 * @note The file is opened in append mode, so existing data is preserved.
 * @warning If the file does not exist, it will be created.
 */
void OdomICP::store_data(const std::vector<Pose>& data) {
    std::string s = WORK_SPACE_PATH + "/../dataset/est_trajectory.txt"; 
    const char* filename = s.c_str(); 
    std::ofstream infile;
    infile.open(filename, std::ios_base::app);
    for (const auto& pose : data) {
        infile << pose.timestamp << " ";
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                infile << pose.pose(i, j) << " ";
            }
        }
        infile << std::endl;
    }
    infile.close();
}
/**
 * @brief Calculate the intersection ratio between two point clouds.
 *
 * This function calculates the intersection ratio between a reference point cloud
 * and a transformed scan point cloud. It does this by finding the bounding boxes
 * of both clouds and computing the volume of their intersection.
 *
 * @return The ratio of the volume of intersection to the volume of the reference cloud.
 * @note The function assumes that the point clouds are already loaded and transformed.
 */
double OdomICP::calc_intersection(){
    //initialize with maxvalues
    double ref_min_x = DBL_MAX, ref_min_y = DBL_MAX, ref_min_z = DBL_MAX;
    double ref_max_x = -DBL_MAX, ref_max_y = -DBL_MAX, ref_max_z = -DBL_MAX;
    double scan_min_x = DBL_MAX, scan_min_y = DBL_MAX, scan_min_z = DBL_MAX;
    double scan_max_x = -DBL_MAX, scan_max_y = -DBL_MAX, scan_max_z = -DBL_MAX;
    //transfrom the current Scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *transformed_scan, Twb.cast<float>());
    // compute bounding box for refCloud
    for(int i = 0; i < refCloud->size(); i++){
        pcl::PointXYZ refcloud_point = refCloud->points[i];
        if(refcloud_point.x < ref_min_x){
            ref_min_x = refcloud_point.x;
        }
        if(refcloud_point.y < ref_min_y){
            ref_min_y = refcloud_point.y;
        }
        if(refcloud_point.z < ref_min_z){
            ref_min_z = refcloud_point.z;
        }
        if(refcloud_point.x > ref_max_x){
            ref_max_x = refcloud_point.x;
        }
        if(refcloud_point.y > ref_max_y){
            ref_max_y = refcloud_point.y;
        }
        if(refcloud_point.z > ref_max_z){
            ref_max_z = refcloud_point.z;
        }
    }
    // compute bounding box for the transformed scan
    for(int i = 0; i < transformed_scan->size(); i++){
        pcl::PointXYZ scancloud_point = transformed_scan->points[i];
        if(scancloud_point.x < scan_min_x){
            scan_min_x = scancloud_point.x;
        }
        if(scancloud_point.y < scan_min_y){
            scan_min_y = scancloud_point.y;
        }
        if(scancloud_point.z < scan_min_z){
            scan_min_z = scancloud_point.z;
        }
        if(scancloud_point.x >scan_max_x){
            scan_max_x = scancloud_point.x;
        }
        if(scancloud_point.y > scan_max_y){
            scan_max_y = scancloud_point.y;
        }
        if(scancloud_point.z > scan_max_z){
            scan_max_z = scancloud_point.z;
        }
    }
    //calculate bounding box for the intersectioin of the two boxes
    double intersection_min_x = std::max(ref_min_x, scan_min_x);
    double intersection_min_y = std::max(ref_min_y, scan_min_y);
    double intersection_min_z = std::max(ref_min_z, scan_min_z);
    double intersection_max_x = std::min(ref_max_x, scan_max_x);
    double intersection_max_y = std::min(ref_max_y, scan_max_y);
    double intersection_max_z = std::min(ref_max_z, scan_max_z);
    //calculate volume of intersection
    double intersection_volume = (intersection_max_x - intersection_min_x) * (intersection_max_y - intersection_min_y) * (intersection_max_z - intersection_min_z);
    //calculate volume of refCloud
    double ref_volume = (ref_max_x - ref_min_x) * (ref_max_y - ref_min_y) * (ref_max_z - ref_min_z);
    std::cout << "Interscetion ratio: " << intersection_volume/ref_volume << std::endl;
    return intersection_volume/ref_volume;
}
/**
 * @brief Remove points from the reference cloud based on Euclidean distance.
 *
 * This function removes points from the reference point cloud that are beyond a specified
 * Euclidean distance from the current position.
 *
 * @note The function assumes that `refCloud` is already initialized and contains valid data.
 * @note The parameter `params::map_range` determines the maximum allowed distance for points
 * to be retained.
 */
void OdomICP::remove_euclidean(){
    // Get current position
    pcl::PointXYZ current_position(Twb(0,3), Twb(1,3), Twb(2,3));
    // Remove points based on eucldiean distance
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for(int i = 0; i < refCloud->size(); i++){
        // compute euclidean distance and compare to map_range
        if(std::sqrt(std::pow(refCloud->points[i].x - current_position.x, 2) + std::pow(refCloud->points[i].y - current_position.y, 2) + std::pow(refCloud->points[i].z - current_position.z, 2)) > params::map_range){
            inliers->indices.push_back(i); 
        }
    }
    extract.setInputCloud(refCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*refCloud);
}
/**
 * @brief Remove points from the reference cloud that fall outside a specified range.
 *
 * This function removes points from the reference point cloud that have coordinates
 * falling outside a specified range around the current position.
 *
 * @note The function assumes that `refCloud` is already initialized and contains valid data.
 * @note The parameter `params::map_range` determines the allowed range around the current position.
 */
void OdomICP::remove_inf(){
    // Get current position
    pcl::PointXYZ current_position(Twb(0,3), Twb(1,3), Twb(2,3));
    // remove all points in refCloud with one coordinate larger than current position + map_range
    pcl::PassThrough<pcl::PointXYZ> pass;
    // filter based on x
    pass.setInputCloud(refCloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(current_position.x - params::map_range, current_position.x + params::map_range);
    pass.filter(*refCloud);
    // filter based on y
    pass.setInputCloud(refCloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(current_position.y - params::map_range, current_position.y + params::map_range);
    pass.filter(*refCloud);
    // filter based on z
    pass.setInputCloud(refCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(current_position.z - params::map_range, current_position.z + params::map_range);
    pass.filter(*refCloud);
}

void OdomICP::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    cloudQueueMutex.lock();
    std_msgs::Header cloudHeader = laserCloudMsg->header;
    cloudHeader.stamp = ros::Time::now();
    cloudQueue.push(std::make_pair(cloudHeader, laserCloudMsg));
    cloudQueueMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr OdomICP::parseCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *cloudTmp);
    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudTmp, *cloudTmp, indices);
    return cloudTmp;
}

void OdomICP::publishResult() {
//    publish odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.header.stamp = cloudHeader.stamp;
    odom.pose.pose.position.x = Twb(0,3);
    odom.pose.pose.position.y = Twb(1,3);
    odom.pose.pose.position.z = Twb(2,3);
    Eigen::Quaterniond q(Twb.block<3,3>(0,0));
    q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_pub.publish(odom);

//    publish path
    path.header.frame_id = "map";
    path.header.stamp = cloudHeader.stamp;
    geometry_msgs::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);
    path_pub.publish(path);

//    publish map
    sensor_msgs::PointCloud2 mapMsg;
    pcl::toROSMsg(*mapCloud, mapMsg); //replaced *refCloud with *mapCloud
    mapMsg.header.frame_id = "map";
    mapMsg.header.stamp = cloudHeader.stamp;
    map_pub.publish(mapMsg);

//    publish laser
    sensor_msgs::PointCloud2 laserMsg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Twb.cast<float>());
    pcl::toROSMsg(*laserTransformed, laserMsg);
    laserMsg.header.frame_id = "map";
    laserMsg.header.stamp = cloudHeader.stamp;
    scan_pub.publish(laserMsg);

    Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2) * 180 / M_PI;
    std::cout << "x: " << Twb(0,3) << " y: " << Twb(1,3) << " z: " << Twb(2,3) << " roll: " << rpy(0) << " pitch: " << rpy(1) << " yaw: " << rpy(2)
    << " time: " << timer.duration_ms() << " ms" << std::endl;
//    traj_file << std::fixed << cloudHeader.stamp.toSec() << " " << Twb(0,0) << " " << Twb(0,1) << " " << Twb(0,2) << " " << Twb(0,3) << " "
//    << Twb(1,0) << " " << Twb(1,1) << " " << Twb(1,2) << " " << Twb(1,3) << " "
//    << Twb(2,0) << " " << Twb(2,1) << " " << Twb(2,2) << " " << Twb(2,3) << std::endl;
}

