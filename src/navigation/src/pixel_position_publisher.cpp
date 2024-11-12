#include "navigation/pixel_position_publisher.h"
#include "bumperbot_utils/utils.h"


// Constructor 
PixelPositionPublisher::PixelPositionPublisher(const ros::NodeHandle &nh, const std::string& map_yaml_path) :
            nh_(nh),
            map_yaml_path_(map_yaml_path)
{
    // Load the YAML map file
    Utils::loadMapYaml(map_yaml_path_, map_resolution_, map_origin_x_, map_origin_y_);

    // Initialize publisher and subscriber
    amcl_pos_sub_ = nh_.subscribe("amcl_pose", 10, &PixelPositionPublisher::msgCallback, this);
    pixel_pub_ = nh_.advertise<geometry_msgs::Point>("robot_pixel_position", 10);
}


// Function: Handles callback when receiving msg from /amcl_pose topic
void PixelPositionPublisher::msgCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Point pixel_msg = Utils::amclToPixel(msg, map_resolution_, map_origin_x_, map_origin_y_);
    // Publish the pixel coordinates to topic
    pixel_pub_.publish(pixel_msg);
}

