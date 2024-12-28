#include <ros/ros.h>
#include <navigation/ConvertPixelToMap.h>
#include <navigation/ConvertPixelWaypointsToMap.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <bumperbot_utils/utils.h>


ros::ServiceClient client;

// Function: Converts a list of pixels coord to list of map waypoints
//           This is required so that waypoint can be passed to ROS Nav.
std::vector<geometry_msgs::Point> convertPixelsToMapWaypoints(
    ros::ServiceClient &client,
    const std::vector<geometry_msgs::Point> &waypoints)
{
    std::vector<geometry_msgs::Point> map_waypoints;
    std::cout << "Converting pixels to map waypoints" << std::endl;

    for (const auto &wp : waypoints) {
        navigation::ConvertPixelToMap srv;
        srv.request.pixel_x = wp.x;
        srv.request.pixel_y = wp.y;

        if (client.call(srv)) {
            geometry_msgs::Point map_point;
            map_point.x = srv.response.real_x;
            map_point.y = srv.response.real_y;
            map_waypoints.push_back(map_point);

            ROS_INFO("Converted Pixel (%f, %f) -> Map Coordinates (%.2f, %.2f)",
                     wp.x, wp.y, srv.response.real_x, srv.response.real_y);
        } else {
            ROS_ERROR("Failed to call service convert_pixel_to_map_server for pixel (%f, %f).", wp.x, wp.y);
        }
    }

    return map_waypoints;
}


// Service callback for ConvertPixelWaypointsToMap
bool convertWaypointsCallback(navigation::ConvertPixelWaypointsToMap::Request &req,
                              navigation::ConvertPixelWaypointsToMap::Response &res)
{   
    // Read the pixel waypoints from the file
    std::vector<geometry_msgs::Point> waypoints = Utils::readWaypointsFromFile(req.waypoint_file);
    if (waypoints.empty()) {
        ROS_ERROR("No waypoints loaded from file.");
        return false;
    }

    // Convert the pixel waypoints to map coordinates
    res.map_waypoints = convertPixelsToMapWaypoints(client, waypoints);
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pixel_waypoints_to_map_waypoints_converter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_;

    // Get Global parameter for topic/services
    std::string convert_pixel_to_map_waypoints_svc_srv_;
    nh_.getParam("/navigation/services/convert_pixel_waypoints_to_map_waypoints", convert_pixel_to_map_waypoints_svc_srv_);
    std::string convert_pixel_to_map_svc_client_;
    nh_.getParam("/navigation/services/convert_pixel_to_map", convert_pixel_to_map_svc_client_);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService(convert_pixel_to_map_waypoints_svc_srv_, convertWaypointsCallback);

    if (!ros::service::waitForService(convert_pixel_to_map_svc_client_, ros::Duration(10.0))) {
        ROS_ERROR("Service %s is not available after waiting", convert_pixel_to_map_svc_client_.c_str());
        return 1;
    }
    // Initialize the service client to call the ConvertPixelToMap service
    client = nh_.serviceClient<navigation::ConvertPixelToMap>(convert_pixel_to_map_svc_client_);

    ROS_INFO("Service to convert pixel waypoints to map waypoints is ready.");
    ros::spin();

    return 0;
}
