#ifndef PIXEL_POSITION_PUBLISHER_H
#define PIXEL_POSITION_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <navigation/GetPixelPose.h>
#include <navigation/GetAmclPose.h>

class PixelPositionPublisher
{
public:
    PixelPositionPublisher(const ros::NodeHandle&, const std::string& map_yaml_path = "");

private:
    void msgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
    bool getPixelPoseCallback(navigation::GetPixelPose::Request &req, navigation::GetPixelPose::Response &res);

    ros::NodeHandle nh_;
    ros::Subscriber amcl_pos_sub_;
    ros::Publisher pixel_pub_;
    ros::ServiceClient client_;
    ros::ServiceServer service_;

    std::string map_yaml_path_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
};

#endif