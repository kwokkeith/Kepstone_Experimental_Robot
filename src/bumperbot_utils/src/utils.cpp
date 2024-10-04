#include "bumperbot_utils/utils.h"

geometry_msgs::Point Utils::amclToPixel(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose, 
                                double map_resolution, double map_origin_x, double map_origin_y)
{
    // Extract the robot's position from the AMCL message
    double x_real = amcl_pose->pose.pose.position.x; // X position in meters
    double y_real = amcl_pose->pose.pose.position.y; // Y position in meters

    // Convert the amcl position coordinates to pixel coordinates
    int x_pixel = static_cast<int>((x_real - map_origin_x) / map_resolution);
    int y_pixel = static_cast<int>((y_real - map_origin_y) / map_resolution);

    geometry_msgs::Point pixel_msg;
    pixel_msg.x = x_pixel;
    pixel_msg.y = y_pixel;

    return pixel_msg;
}

// Function: Loads YAML file of map created from gmapping
bool Utils::loadMapYaml(const std::string& yaml_path, double& map_resolution, double& map_origin_x, double& map_origin_y)
{
    ROS_INFO("Attempting to load map from: %s", yaml_path.c_str());
    try {
        YAML::Node map_data = YAML::LoadFile(yaml_path);
        
        // Extract resolution and origin from the YAML file
        map_resolution = map_data["resolution"].as<double>();
        map_origin_x = map_data["origin"][0].as<double>();
        map_origin_y = map_data["origin"][1].as<double>();

        ROS_INFO("Map loaded: resolution = %f, origin = (%f, %f)", map_resolution, map_origin_x, map_origin_y);
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load YAML file: %s", e.what());
        return false;
    }
}