#include "odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include "global_definition.h"

using namespace std;
using namespace Eigen;


OdomICP::~OdomICP() {}

OdomICP::OdomICP(ros::NodeHandle &nh):
        nh_(nh) {
//    initialize variables here
    Twb = Eigen::Matrix4d::Identity(); // initial pose
    Twk = Eigen::Matrix4d::Identity(); // initial key frame
    Twb_gt = Eigen::Matrix4d::Identity(); // ground truth pose
    Twb_prev = Eigen::Matrix4d::Identity(); // previous pose
    deltaT_pred = Eigen::Matrix4d::Identity(); // predicted pose change

    firstFrame = true;

    laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>);
    refCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    prevCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

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
    std::cout << "Odometry ICP initialized" << std::endl;
}

void OdomICP::run() {
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
            Twb = Eigen::Matrix4d::Identity();
            *refCloud = *laserCloudIn;
            continue;
        }

        timer.tic();
        // Odometry estimation
        // 0. save previous pose
        Twb_prev = Twb;

        // 1. preprocess: downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        //Filter Laser Scan cloud
        dsFilterScan.setInputCloud(laserCloudIn);
        dsFilterScan.filter(*laserCloud_filtered);
        std::cout << "filtered Scan size: " << laserCloud_filtered->size();
        //Filter Map Cloud
        dsFilterMap.setInputCloud(refCloud);
        dsFilterMap.filter(*refCloud_filtered);
        std:cout << "filtered Map size: " << refCloud_filtered->size();
        
        // 2. icp
        // 3. update pose
        //deltaT_pred = icp_registration(laserCloud_filtered, refCloud_filtered, deltaT_pred);
        //deltaT_pred = icp_registration(refCloud_filtered, laserCloud_filtered, Eigen::Matrix4d::Identity());
        //Twb *= deltaT_pred;
        
        //Twb *= icp_registration(laserCloud_filtered, refCloud_filtered, Twb); 
        
        // 4. update reference cloud
        //pcl::transformPointCloud(*refCloud, *refCloud, deltaT_pred);
        //add the current laserCloud to    the ref point cloud
        *refCloud += *laserCloud_filtered;
        // extract current positioin in the final transfromation matrix
        Eigen::Vector3d t = Twb.block<3,1>(0,3);
        // Delete all points in the ref cloud that are too far away from the current position print old size
        std::cout << "refCloud size: " << refCloud->size() << std::endl;
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(refCloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(t(0) - 50, t(0) + 50);
        pass.filter(*refCloud);
        pass.setInputCloud(refCloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(t(1) - 50, t(1) + 50);
        pass.filter(*refCloud);
        pass.setInputCloud(refCloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(t(2) - 50, t(2) + 50);
        pass.filter(*refCloud);
        std::cout << "refCloud size: " << refCloud->size() << std::endl;
        //downsample the current map
        dsFilterMap.setInputCloud(refCloud);
        dsFilterMap.filter(*refCloud);
        std::cout << "refCloud size: " << refCloud->size() << std::endl;
        

        
        Twb = icp_registration2(laserCloud_filtered, prevCloud, Twb);
        std::cout << "Should be not identity matrix: " << Twb << std::endl;

        timer.toc();
        // 5. publish result
        publishResult();
        rate.sleep();
        prevCloud = laserCloud_filtered;
    }
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
    pcl::toROSMsg(*refCloud, mapMsg);
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

