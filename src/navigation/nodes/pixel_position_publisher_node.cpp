#include <ros/ros.h>
#include "navigation/pixel_position_publisher.h"
#include <string>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_position_publisher_cpp");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle 
    
    std::string map_yaml_path;
    pnh.getParam("map_yaml_path", map_yaml_path);
    
    PixelPositionPublisher publisher(nh, map_yaml_path);

    ros::spin();

    return 0;
}