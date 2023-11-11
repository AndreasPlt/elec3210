#include "ekf_slam.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;


EKFSLAM::~EKFSLAM() {}

EKFSLAM::EKFSLAM(ros::NodeHandle &nh):
        nh_(nh) {

//    initialize ros publisher
    lidar_sub = nh_.subscribe("/velodyne_points", 1, &EKFSLAM::cloudHandler, this);
    odom_sub = nh_.subscribe("/odom", 1, &EKFSLAM::odomHandler, this);
    map_cylinder_pub = nh_.advertise<visualization_msgs::MarkerArray>("/map_cylinder", 1);
    obs_cylinder_pub = nh_.advertise<visualization_msgs::MarkerArray>("/obs_cylinder", 1);
    odom_pub = nh_.advertise<nav_msgs::Odometry>("ekf_odom", 1000);
    path_pub = nh_.advertise<nav_msgs::Path>("ekf_path", 1000);
    scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("current_scan", 1);
    map_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
    laserCloudIn = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    mapCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    extractCylinder = std::make_shared<ExtractCylinder>(nh_);

    globalId = -1;
	/**
	 * TODO: initialize the state vector and covariance matrix
	 */
    mState = Eigen::VectorXd::Zero(3); // x, y, yaw
    mCov = Eigen::MatrixXd::Zero(3, 3);
    R = 0.01 * Eigen::MatrixXd::Identity(3, 3); // process noise
    Q = Eigen::MatrixXd::Zero(2, 2); // measurement nosie
    Q << 0.01, 0, 0, 0.01;

    std::cout << "EKF SLAM initialized" << std::endl;
}

void EKFSLAM::run() {
    ros::Rate rate(1000);
    while (ros::ok()){
        if (cloudQueue.empty() || odomQueue.empty()){
            rate.sleep();
            continue;
        }

        cloudQueueMutex.lock();
        cloudHeader = cloudQueue.front().first;
        laserCloudIn = parseCloud(cloudQueue.front().second);
        cloudQueue.pop();
        cloudQueueMutex.unlock();

        // find the cloest odometry message
        odomMutex.lock();
        auto odomIter = odomQueue.front();
        auto odomPrevIter = odomQueue.front();
        while (!odomQueue.empty() && odomIter != odomQueue.back() && odomIter.first.stamp < cloudHeader.stamp){
            odomPrevIter = odomIter;
            odomIter = odomQueue.front();
            odomQueue.pop();
        }
        odomMutex.unlock();

        if (firstFrame){
            firstFrame = false;
            Twb = Eigen::Matrix4d::Identity();
            cloudHeaderLast = cloudHeader;
            continue;
        }

        auto odomMsg = odomIter == odomQueue.back() ? odomPrevIter : odomIter;
        Eigen::Vector2d ut = Eigen::Vector2d(odomMsg.second->twist.twist.linear.x, odomMsg.second->twist.twist.angular.z);
        double dt = (cloudHeader.stamp - cloudHeaderLast.stamp).toSec();

        timer.tic();
		// Extended Kalman Filter
		// 1. predict
        predictState(mState, mCov, ut, dt);
		// 2. update
        updateMeasurement();
		timer.toc();

		// publish odometry and map
		map_pub_timer.tic();
        accumulateMap();
        publishMsg();
		cloudHeaderLast = cloudHeader;

        rate.sleep();
    }
}

double EKFSLAM::normalizeAngle(double angle){
	if (angle > M_PI){
		angle -= 2 * M_PI;
	} else if (angle < -M_PI){
		angle += 2 * M_PI;
	}
	return angle;
}

Eigen::Matrix3d EKFSLAM::jacobGxt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::Matrix3d Gt = Eigen::Matrix3d::Zero();

    double v = ut(0);
    double w = ut(1);
    double theta = state(2);

    Gt(0, 2) = -v/w * cos(theta) + v/w * cos(theta + w * dt);
    Gt(1, 2) = -v/w * sin(theta) + v/w * sin(theta + w * dt);

	return Gt;
}

Eigen::MatrixXd EKFSLAM::jacobFt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(num_state, 2);
	/**
	 * TODO: implement the Jacobian Ft
	 */
	return Ft;
}

Eigen::MatrixXd EKFSLAM::jacobB(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt){
	int num_state = state.rows();
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_state, 2);
	B(0, 0) = dt * cos(state(2));
	B(1, 0) = dt * sin(state(2));
	B(2, 1) = dt;
	return B;
}

void EKFSLAM::predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov, Eigen::Vector2d ut, double dt){
	// Note: ut = [v, w]
    int num_state = state.rows();
    Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(3, num_state);
    Fx.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

    double v = ut(0);
    double w = ut(1);
    double theta = state(2);

    // update state
    Eigen::Vector3d state_change = Eigen::Vector3d::Zero();
    state_change(0) = -v/w * sin(theta) + v/w * sin(theta + w * dt);
    state_change(1) = v/w * cos(theta) - v/w * cos(theta + w * dt);
    state_change(2) = w * dt;
    // print dimensions
    mState = state + Fx.transpose() * state_change;
    mState(2,0) = normalizeAngle(mState(2,0));
    double new_theta = state(2, 0);
    if (abs(new_theta - theta) > 0.8){
        std::cout << "[UPDATE] theta increase!" << std::endl;
    }

    // update covariance
    // print dimensions
	Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(num_state, num_state) + Fx.transpose() * jacobGxt(state, ut, dt) * Fx;
    Eigen::Matrix3d Rx = R.block(0, 0, 3, 3);
    mCov = Gt * cov * Gt.transpose() + Fx.transpose() * Rx * Fx; // update covariance
}

Eigen::Vector2d EKFSLAM::transform(const Eigen::Vector2d& p, const Eigen::Vector3d& x){
	Eigen::Vector2d p_t;
	p_t(0) = p(0) * cos(x(2)) - p(1) * sin(x(2)) + x(0);
	p_t(1) = p(0) * sin(x(2)) + p(1) * cos(x(2)) + x(1);
	return p_t;
}

void EKFSLAM::addNewLandmark(const Eigen::Vector2d& lm, const Eigen::MatrixXd& InitCov){
	// add new landmark to mState and mCov
	/**
	 * TODO: check functon
	 */

    int state_size = mState.rows();

    Eigen::VectorXd new_state = Eigen::VectorXd::Zero(state_size + 2);
    new_state.segment(0, state_size) = mState;
    new_state.segment(state_size, 2) = lm;
    mState = new_state;

    Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(state_size + 2, state_size + 2);
    new_cov.block(0, 0, state_size, state_size) = mCov;
    new_cov.block(state_size, state_size, 2, 2) = InitCov;
    mCov = new_cov;

    Eigen::MatrixXd new_process_noise = Eigen::MatrixXd::Zero(state_size + 2, state_size + 2);
    new_process_noise.block(0, 0, state_size, state_size) = R;
    new_process_noise.block(state_size, state_size, 2, 2) = Q;
    R = new_process_noise;    
}

void EKFSLAM::accumulateMap(){

    Eigen::Matrix4d Twb = Pose3DTo6D(mState.segment(0, 3));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *transformedCloud, Twb);
    *mapCloud += *transformedCloud;

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(mapCloud);
    voxelSampler.setLeafSize(0.5, 0.5, 0.5);
    voxelSampler.filter(*mapCloud);
}

void EKFSLAM::updateMeasurement(){
    cylinderPoints = extractCylinder->extract(laserCloudIn, cloudHeader); // 2D pole centers in the laser/body frame
    Eigen::Vector3d xwb = mState.block<3, 1>(0, 0); // pose in the world frame
    int num_landmarks = (mState.rows() - 3) / 2; // number of landmarks in the state vector
    int num_obs = cylinderPoints.rows(); // number of observations
    Eigen::VectorXi indices = Eigen::VectorXi::Ones(num_obs) * -1; // indices of landmarks in the state vector
    
    std::vector<int> observations(num_obs); // indices of observations that not already mapped
    std::iota(observations.begin(), observations.end(), 0); // fill with 0, 1, 2, ..., num_obs - 1
    
    // iterate through all landmarks
    for (int j = 0; j < num_landmarks; ++j) {
        // get current coordinates in world frame
        Eigen::Vector2d landmark = mState.block<2, 1>(3 + j * 2, 0);
        int min_idx = -1;
        double min_dist = 99999;
        // iterate through all observations
        for (int i = 0; i < num_obs; ++i) {
            // get current coordinates in world frame
            Eigen::Vector2d pt_transformed = transform(cylinderPoints.row(i).transpose(), xwb);
            // calc euclidean distance between the points
            double dist = (pt_transformed - landmark).norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = i;
            }
        }
        // if the closest observation is closer than the threshold, we assume that the observation is the same landmark
        if (min_dist < DATA_ASSOCIATION_THRESHOLD && min_idx != -1) {
            indices(min_idx) = j;
            observations.erase(
                std::remove(observations.begin(), observations.end(), min_idx), 
                observations.end());
        }
    }

    for (int i = 0; i < num_obs; ++i) {
        
        if (indices(i) == -1){
            indices(i) = ++globalId;
            Eigen::Vector2d pt_transformed = transform(cylinderPoints.row(i).transpose(), xwb);
            addNewLandmark(pt_transformed, Q);
        }
    }

    // simulating bearing model
    Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * num_obs);
    for (int i = 0; i < num_obs; ++i) {
        const Eigen::Vector2d& pt = cylinderPoints.row(i);
        z(2 * i, 0) = pt.norm();
        z(2 * i + 1, 0) = atan2(pt(1), pt(0));
    }
    // update the measurement vector
    num_landmarks = (mState.rows() - 3) / 2;
    for (int i = 0; i < num_obs; ++i) {
        int idx = indices(i);
        if (idx == -1 || idx + 1 > num_landmarks) continue;
        const Eigen::Vector2d& landmark = mState.block<2, 1>(3 + idx * 2, 0);
		// Implement the measurement update here, i.e., update the state vector and covariance matrix
		/**
		 * TODO: check measurement update
		 */
        // get z_hat
        Eigen::Vector2d delta = landmark - mState.block<2, 1>(0, 0);
        double q = delta.transpose() * delta;
        double state_theta = mState(2, 0);
        Eigen::Vector2d z_hat = Eigen::Vector2d(sqrt(q), normalizeAngle(atan2(delta(1), delta(0)) - state_theta));
        
        // get F_xj
        Eigen::MatrixXd F_xj = Eigen::MatrixXd::Zero(5, mState.rows());
        F_xj.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
        F_xj.block(3, 3 + idx * 2, 2, 2) = Eigen::MatrixXd::Identity(2, 2);

        // get jacobian of measurement model
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, mState.rows());
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(2, 5);
        Eigen::Vector2d pt = cylinderPoints.row(i);

        double delta_x = delta(0);
        double delta_y = delta(1);

        H_i << -delta_x / sqrt(q), -delta_y / sqrt(q), 0, delta_x / sqrt(q), delta_y / sqrt(q),
                delta_y / q, -delta_x / q, -1, -delta_y / q, delta_x / q;
        H = H_i * F_xj;

        // get kalman gain
        Eigen::MatrixXd K = mCov * H.transpose() * (H * mCov * H.transpose() + Q).inverse();
        
        // update state
        Eigen::Vector2d z_diff = z.block<2, 1>(2 * i, 0) - z_hat;
        mState += K * z_diff;
        double new_theta = mState(2, 0);
        if (abs(new_theta - state_theta) > 0.8){
            std::cout << "[UPDATE] theta increase!" << std::endl;
        }

        // update covariance
        mCov = (Eigen::MatrixXd::Identity(mState.rows(), mState.rows()) - K * H) * mCov;
    }
}

void EKFSLAM::publishMsg(){
    // publish map cylinder
    visualization_msgs::MarkerArray markerArray;
    int num_landmarks = (mState.rows() - 3) / 2;
    for (int i = 0; i < num_landmarks; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = cloudHeader.stamp;
        marker.ns = "map_cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mState(3 + i * 2, 0);
        marker.pose.position.y = mState(3 + i * 2 + 1, 0);
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }
    map_cylinder_pub.publish(markerArray);

    int num_obs = cylinderPoints.rows();
    markerArray.markers.clear();
    for (int i = 0; i < num_obs; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = cloudHeader.stamp;
        marker.ns = "obs_cylinder";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        Eigen::Vector2d pt = transform(cylinderPoints.row(i).transpose(), mState.segment(0, 3));
        marker.pose.position.x = pt(0);
        marker.pose.position.y = pt(1);
        marker.pose.position.z = 0.5;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        markerArray.markers.push_back(marker);
    }
    obs_cylinder_pub.publish(markerArray);

//    publish odom
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.header.stamp = cloudHeader.stamp;
    odom.pose.pose.position.x = mState(0,0);
    odom.pose.pose.position.y = mState(1,0);
    odom.pose.pose.position.z = 0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(mState(2,0), Eigen::Vector3d::UnitZ()));
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

////    publish map
    sensor_msgs::PointCloud2 mapMsg;
    pcl::toROSMsg(*mapCloud, mapMsg);
    mapMsg.header.frame_id = "map";
    mapMsg.header.stamp = cloudHeader.stamp;
    map_pub.publish(mapMsg);

////    publish laser
    sensor_msgs::PointCloud2 laserMsg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Pose3DTo6D(mState.segment(0, 3)).cast<float>());
    pcl::toROSMsg(*laserTransformed, laserMsg);
    laserMsg.header.frame_id = "map";
    laserMsg.header.stamp = cloudHeader.stamp;
    scan_pub.publish(laserMsg);

	map_pub_timer.toc();
    std::cout << "x: " << mState(0,0) << " y: " << mState(1,0) << " theta: " << mState(2,0) * 180 / M_PI << " obs/landmarks: " << num_obs << "/" << num_landmarks << ", time ekf: " << timer.duration_ms() << " ms"
						  << ", map_pub: " << map_pub_timer.duration_ms() << " ms" << std::endl;
}

void EKFSLAM::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    cloudQueueMutex.lock();
    std_msgs::Header cloudHeader = laserCloudMsg->header;
    cloudQueue.push(std::make_pair(cloudHeader, laserCloudMsg));
    cloudQueueMutex.unlock();
}

void EKFSLAM::odomHandler(const nav_msgs::OdometryConstPtr& odomMsg){
    odomMutex.lock();
    std_msgs::Header odomHeader = odomMsg->header;
    odomQueue.push(std::make_pair(odomHeader, odomMsg));
    odomMutex.unlock();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EKFSLAM::parseCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTmp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserCloudMsg, *cloudTmp);
    // Remove Nan points
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloudTmp, *cloudTmp, indices);
    return cloudTmp;
}

