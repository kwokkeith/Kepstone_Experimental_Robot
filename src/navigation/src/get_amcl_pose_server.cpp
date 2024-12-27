#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <navigation/GetAmclPose.h>  

geometry_msgs::PoseWithCovarianceStamped latest_amcl_pose;
bool has_new_pose = false;

// Callback to update the AMCL pose when a new message is received
void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) 
{
    latest_amcl_pose = *msg;
    has_new_pose = true;
}

// Service callback to return the latest AMCL pose
bool getAmclPoseCallback(navigation::GetAmclPose::Request &req, navigation::GetAmclPose::Response &res)
{
    if (has_new_pose) {
        res.pose = latest_amcl_pose;
        return true;
    } else {
        ROS_WARN("No AMCL pose available yet.");
        return false;
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "get_amcl_pose_server");
    ros::NodeHandle nh;
    // Load global parameters
    std::string get_amcl_pose_srv_;
    nh.getParam("/navigation/services/get_amcl_pose", get_amcl_pose_srv_);
    std::string get_amcl_pose_sub_;
    nh.getParam("/navigation/topics/get_amcl_pose", get_amcl_pose_sub_);

    // Subscribe to the AMCL pose topic
    ros::Subscriber amcl_sub = nh.subscribe(get_amcl_pose_sub_, 10, amclPoseCallback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService(get_amcl_pose_srv_, getAmclPoseCallback);

    ROS_INFO("Service to provide AMCL pose is ready.");

    ros::spin();

    return 0;
}
