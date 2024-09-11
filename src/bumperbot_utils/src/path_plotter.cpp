#include "bumperbot_utils/path_plotter.h" 
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>


PathPlotter::PathPlotter(const ros::NodeHandle &nh) :
                        nh_(nh)
{
    ROS_INFO("Listening to odometry and plotting");
    odom_sub_ = nh_.subscribe("bumperbot_controller/odom", 10, &PathPlotter::odomCallback, this);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("bumperbot_controller/trajectory", 10);
}

void PathPlotter::odomCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg.header.frame_id;
    pose.header.stamp = msg.header.stamp;
    
    pose.pose.position.x = msg.pose.pose.position.x;
    pose.pose.position.y = msg.pose.pose.position.y;
    pose.pose.position.z = msg.pose.pose.position.z;

    pose.pose.orientation.x = msg.pose.pose.orientation.x;
    pose.pose.orientation.y = msg.pose.pose.orientation.y;
    pose.pose.orientation.z = msg.pose.pose.orientation.z;
    pose.pose.orientation.w = msg.pose.pose.orientation.w;

    poses.push_back(pose);
    nav_msgs::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = ros::Time::now();
    path.poses = poses;

    trajectory_pub_.publish(path);
}