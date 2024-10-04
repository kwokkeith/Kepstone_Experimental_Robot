#ifndef UTILS_H
#define UTILS_H
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <yaml-cpp/yaml.h>

class Utils
{
public:
    static geometry_msgs::Point amclToPixel(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose, double map_resolution, double map_origin_x, double map_origin_y);
    static bool loadMapYaml(const std::string& yaml_path, double& map_resolution, double& map_origin_x, double& map_origin_y);

};

#endif