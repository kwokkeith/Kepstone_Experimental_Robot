#ifndef PATH_PLOTTER_H
#define PATH_PLOTTER_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class PathPlotter
{
public:
    PathPlotter(const ros::NodeHandle &);

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher trajectory_pub_;

    std::vector<geometry_msgs::PoseStamped> poses;

    void odomCallback(const nav_msgs::Odometry &);
};

#endif