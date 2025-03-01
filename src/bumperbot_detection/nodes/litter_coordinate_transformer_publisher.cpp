#include <ros/ros.h>
#include "bumperbot_detection/litter_coordinate_transformer.h"


int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "litter_coordinate_transformer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh_("~");
    std::string base_frame_;
    std::string camera_frame_;
    std::string rear_camera_frame_;

    // Load frame names from parameters
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");     // Default base frame/link = "base_footprint"
    pnh_.param<std::string>("camera_frame", camera_frame_, "dpcamera_link");  // Default camera frame/link = "dpcamera_link"
    pnh_.param<std::string>("rear_camera_frame", rear_camera_frame_, "rear_dpcamera_link");  // Default camera frame/link = "dpcamera_link"

    // Create the transformer object
    LitterCoordinateTransformer transformer(nh, base_frame_, camera_frame_, rear_camera_frame_);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}