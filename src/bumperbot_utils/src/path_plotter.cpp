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

// Use ConstPtr (boost shared pointer) so that msg is not copied but a pointer is
// just passed around. msg is READONLY!
void PathPlotter::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = msg->header.frame_id;
    pose.header.stamp = msg->header.stamp;
    
    pose.pose.position.x = msg->pose.pose.position.x;
    pose.pose = msg->pose.pose;


    path.header.frame_id = msg->header.frame_id;
    path.poses.push_back(pose);

    trajectory_pub_.publish(path);
}