#include "bumperbot_utils/path_plotter.h" 
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>


PathPlotter::PathPlotter(const ros::NodeHandle &nh) :
                        nh_(nh)
{
    // Get Global Parameters for topic/services
    std::string amcl_pose_topic_sub_;
    nh_.getParam("/navigation/topics/get_amcl_pose", amcl_pose_topic_sub_);
    std::string robot_trajectory_topic_pub_;
    nh_.getParam("/bumperbot_controller/topics/trajectory", robot_trajectory_topic_pub_);


    ROS_INFO("Listening to odometry and plotting");
    odom_sub_ = nh_.subscribe(amcl_pose_topic_sub_, 10, &PathPlotter::amclCallback, this);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>(robot_trajectory_topic_pub_, 10);
}

// Use ConstPtr (boost shared pointer) so that msg is not copied but a pointer is
// just passed around. msg is READONLY!
void PathPlotter::amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg.header.stamp;
    
    pose.pose.position.x = msg.pose.pose.position.x;
    pose.pose.position.y = msg.pose.pose.position.y;
    pose.pose.orientation = msg.pose.pose.orientation;

    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    path.poses.push_back(pose);

    trajectory_pub_.publish(path);
}