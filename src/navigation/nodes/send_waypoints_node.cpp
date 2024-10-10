#include <ros/ros.h>
#include "navigation/send_waypoints_to_navstack.h"
#include "navigation/ConvertPixelWaypointsToMap.h"
#include "bumperbot_utils/utils.h"
#include <nav_msgs/Path.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "send_waypoints_navstack");

    // Check if file path argument is provided
    if (argc < 2) {
        ROS_ERROR("Usage: rosrun <package_name> send_waypoints_navstack <waypoints_file_path>");
        return 1;
    }

    std::string waypoints_file = argv[1];

    // Define the action client
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to start
    ROS_INFO("Waiting for the move_base action server to come up...");
    ac.waitForServer();

    // Load waypoints (pixels) from the file path (waypoints txt file)
    std::vector<geometry_msgs::Point> map_waypoints = Utils::readWaypointsFromFile(waypoints_file);

    // Initialize the service client for converting pixels to map coordinates
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<navigation::ConvertPixelWaypointsToMap>("convert_pixel_waypoints_to_map_waypoints");

    // Prepare the service request and response
    navigation::ConvertPixelWaypointsToMap srv;
    srv.request.waypoint_file = waypoints_file;

    // Call the service to convert pixel waypoints to map waypoints
    if (client.call(srv)) {
        ROS_INFO("Successfully converted pixel waypoints to map waypoints.");

        // Send the converted map waypoints to the navigation stack
        if (!srv.response.map_waypoints.empty()) {
            
            sendWaypointsToNavStack(srv.response.map_waypoints, ac);
        } else {
            ROS_ERROR("No map waypoints were loaded after conversion.");
        }
    } else {
        ROS_ERROR("Failed to call service to convert pixel waypoints to map waypoints.");
        return 1;
    }

    return 0;
}