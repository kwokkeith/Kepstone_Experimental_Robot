#include <ros/ros.h>
#include <bumperbot_utils/utils.h> 
#include <navigation/ConvertPixelToMap.h>

using namespace std;


double map_resolution_;
double map_origin_x_;
double map_origin_y_;
std::string map_yaml_path_;

bool convertPixelToMap(navigation::ConvertPixelToMap::Request &req,
                       navigation::ConvertPixelToMap::Response &res) 
{
    ROS_INFO("--- Loaded Map Metadata ---");
    ROS_INFO("Map Yaml Path: %s", map_yaml_path_.c_str());
    ROS_INFO("Map Resolution: %f", map_resolution_);
    ROS_INFO("Map Origin: %f, %f", map_origin_x_, map_origin_y_);

    // Convert pixel coordinates to real-world coordinates using the utility function
    float real_x = req.pixel_x * map_resolution_ + map_origin_x_;
    float real_y = req.pixel_y * map_resolution_ + map_origin_y_;

    // float real_x = (req.pixel_x - 100 ) * 0.025;
    // float real_y = (req.pixel_y - 2000 ) * 0.025;


    res.real_x = real_x;
    res.real_y = real_y;

    ROS_INFO("Converted Pixel (%d, %d) -> Real-world (%.2f, %.2f)", req.pixel_x, req.pixel_y, real_x, real_y);

    return true;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "pixel_to_map_converter");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle 

    // Retrieve the YAML file path from the parameter server
    pnh.getParam("map_yaml_path", map_yaml_path_);

    // Load the YAML map file
    Utils::loadMapYaml(map_yaml_path_, map_resolution_, map_origin_x_, map_origin_y_);

    // Get Global parameter for topic/services
    std::string convert_pixel_to_map_svc_srv_;
    nh.getParam("/navigation/services/convert_pixel_to_map", convert_pixel_to_map_svc_srv_);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService(convert_pixel_to_map_svc_srv_, convertPixelToMap);

    ROS_INFO("Service ready to convert pixel to map coordinates.");

    ros::spin();

    return 0;
}
