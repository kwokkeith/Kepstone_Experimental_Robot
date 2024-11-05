#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class KalmanFilter
{
public:
    KalmanFilter(const ros::NodeHandle &nh);
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher amcl_initial_pose_pub_;
    
    // Variables for tracking odometry, IMU, and LiDAR data
    double angular_mean_;
    double angular_variance_;
    double position_mean_;
    double position_variance_;
    double imu_angular_z_;
    double lidar_pos_;
    bool is_first_odom_;
    double last_angular_z_;
    bool odometry_enabled_;
    
    // Store the last known pose for relocalization
    geometry_msgs::Pose last_known_pose_;  // Add this variable to store last known pose

    // Variance values
    double motion_variance_;
    double measurement_variance_;

    // Callback functions
    void imuCallback(const sensor_msgs::Imu& imu);
    void odomCallback(const nav_msgs::Odometry& odom);
    void lidarCallback(const sensor_msgs::LaserScan& scan);

    // State and measurement updates
    void angularMeasurementUpdate();
    void angularStatePrediction(double angular_motion);
    void positionMeasurementUpdate();

    // Relocalization and odometry control
    void triggerRelocalization();
    void recoverAMCLParticles();
    void enableOdometry(bool enable);
};

#endif