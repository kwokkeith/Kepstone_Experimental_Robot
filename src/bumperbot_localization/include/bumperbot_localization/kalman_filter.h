#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


class KalmanFilter
{
public:
    KalmanFilter(const ros::NodeHandle &);

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;

    double mean_;
    double variance_;
    double imu_angular_z_;
    bool is_first_odom_;
    double last_angular_z_;
    double motion_;
    nav_msgs::Odometry kalman_odom_;
    double motion_variance_;
    double measurement_variance_;


    void odomCallback(const nav_msgs::Odometry&);
    void imuCallback(const sensor_msgs::Imu&);

    void measurementUpdate();
    
    void statePrediction();

};
#endif