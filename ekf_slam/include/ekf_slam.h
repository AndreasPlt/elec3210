#ifndef SRC_ICP_ODOM_H
#define SRC_ICP_ODOM_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <nav_msgs/Odometry.h>

#include <numeric>
#include <vector>
#include <queue>
#include <mutex>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>

#include "extract_cylinder.h"
#include "utils.h"

class EKFSLAM {

private:
    ros::NodeHandle &nh_;
    ros::Subscriber lidar_sub, odom_sub;
    ros::Publisher map_cylinder_pub, obs_cylinder_pub, odom_pub, path_pub, map_pub, scan_pub;
    nav_msgs::Path path;
    std::queue<std::pair<std_msgs::Header, sensor_msgs::PointCloud2ConstPtr>> cloudQueue;
    std::queue<std::pair<std_msgs::Header, nav_msgs::Odometry::ConstPtr>> odomQueue;
    std_msgs::Header cloudHeader, cloudHeaderLast;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn, mapCloud;

    Eigen::Matrix4d Twb; // body to world
    Eigen::VectorXd mState; // x, y, yaw, l1, ..., lN (l_x, l_y)
    Eigen::MatrixXd mCov; // covariance matrix of mState
    Eigen::MatrixXd R; // process noise
    Eigen::MatrixXd Q; // measurement noise

    bool firstFrame = true;
    std::shared_ptr<ExtractCylinder> extractCylinder;
    Eigen::MatrixX2d cylinderPoints;

    int globalId = -1;

    TicToc timer, map_pub_timer;

    std::mutex cloudQueueMutex, odomMutex;

public:

    EKFSLAM(ros::NodeHandle &n);

    ~EKFSLAM();

    void run();

    double normalizeAngle(double angle);

    Eigen::MatrixXd jacobGt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    
    Eigen::MatrixXd jacobFt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    
    Eigen::MatrixXd jacobB(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    
    void predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov, Eigen::Vector2d ut, double dt);

    Eigen::Vector2d transform(const Eigen::Vector2d& p, const Eigen::Vector3d& x);

    void accumulateMap();

    void updateMeasurement();

    void addNewLandmark(const Eigen::Vector2d& lm, const Eigen::MatrixXd& InitCov);

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

    void odomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg);

    void publishMsg();

    Eigen::MatrixXd calc_H(const Eigen::Vector2d& delta);

    Eigen::MatrixXd calc_F(int rows, int idx);

    pcl::PointCloud<pcl::PointXYZ>::Ptr parseCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    
    // getters/setters for testing
    Eigen::VectorXd getState() { return mState; }
    Eigen::MatrixXd getCov() { return mCov; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getMapCloud() { return mapCloud; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLaserCloudIn() { return laserCloudIn; }
    Eigen::Matrix4d getTwb() { return Twb; }
    Eigen::MatrixXd getR() { return R; }
    Eigen::MatrixXd getQ() { return Q; }

    void setState(Eigen::VectorXd& state) { mState = state; }
    void setCov(Eigen::MatrixXd& cov) { mCov = cov; }
    void setMapCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& mapCloud) { this->mapCloud = mapCloud; }
    void setLaserCloudIn(pcl::PointCloud<pcl::PointXYZ>::Ptr& laserCloudIn) { this->laserCloudIn = laserCloudIn; }  
    void setTwb(Eigen::Matrix4d& Twb) { this->Twb = Twb; }
    void setR(Eigen::MatrixXd& R) { this->R = R; }
    void setQ(Eigen::MatrixXd& Q) { this->Q = Q; }
};
#endif