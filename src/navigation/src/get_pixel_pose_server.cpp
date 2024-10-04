#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <navigation/GetAmclPose.h>
#include <navigation/GetPixelPose.h>
#include <bumperbot_utils/utils.h>

std::string map_yaml_path_;
double map_resolution_;
double map_origin_x_;
double map_origin_y_;
ros::ServiceClient client_;
ros::ServiceServer service_;

// Function: To handle a service to send pixel coordinate of amcl pose
bool getPixelPoseCallback(navigation::GetPixelPose::Request &req, navigation::GetPixelPose::Response &res)
{   
    // Declare service request/response
    navigation::GetAmclPose srv;

    // Call get_amcl_pose service to get latest amcl position
    if(client_.call(srv))
    {
        geometry_msgs::PoseWithCovarianceStamped pose = srv.response.pose;
        geometry_msgs::Point pixel_pose = Utils::amclToPixel(
                                        boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(pose), 
                                        map_resolution_, 
                                        map_origin_x_, 
                                        map_origin_y_
                                        );
        res.pixel_pose = pixel_pose;
        return true;
    } else {
        ROS_ERROR("Failed to call service get_amcl_pose");
        return false;
    }
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "get_pixel_pose_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle 

    // Retrieve the YAML file path from the parameter server
    pnh.getParam("map_yaml_path", map_yaml_path_);

    // Load the YAML map file
    Utils::loadMapYaml(map_yaml_path_, map_resolution_, map_origin_x_, map_origin_y_);

    // Initialize client for amcl pose
    client_ = nh.serviceClient<navigation::GetAmclPose>("get_amcl_pose");

    // Initialize server for pixel pose
    service_ = nh.advertiseService("get_pixel_pose", getPixelPoseCallback);

    ROS_INFO("Service to provide pixel pose is ready.");

    ros::spin();

    return 0;
}
