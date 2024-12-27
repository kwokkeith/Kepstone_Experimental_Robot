#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <navigation/ConvertPixelWaypointsToMap.h>  
#include <tf/tf.h>


std::vector<geometry_msgs::PoseStamped> poses;
std::string base_frame = "map";

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_plotter_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Private node handle 

    // Retrieve the waypoints file path from ROS parameter server
    std::string waypoints_file;
    if (!pnh.getParam("waypoints_file", waypoints_file)) {
        ROS_ERROR("Failed to get waypoints_file parameter");
        return 1;
    }
  
    std::string convert_pixel_to_map_waypoints_srv_client;
    if (!nh.getParam("/navigation/services/convert_pixel_waypoints_to_map_waypoints", convert_pixel_to_map_waypoints_srv_client)) {
        ROS_ERROR("Failed to get parameter for service convert_pixel_waypoints_to_map_waypoints");
        return 1;
    }
    std::string map_waypoint_path_topic_pub_;
    if (!nh.getParam("/navigation/topics/map_waypoint_path", map_waypoint_path_topic_pub_)) {
        ROS_ERROR("Failed to get parameter for topic map_waypoint_path");
        return 1;
    }

    // Wait for the service to become available
    ROS_INFO("Waiting for service %s to become available...", convert_pixel_to_map_waypoints_srv_client.c_str());
    if (!ros::service::waitForService(convert_pixel_to_map_waypoints_srv_client, ros::Duration(10.0))) {
        ROS_ERROR("Service %s is not available after waiting", convert_pixel_to_map_waypoints_srv_client.c_str());
        return 1;
    }
    ROS_INFO("Service %s is now available.", convert_pixel_to_map_waypoints_srv_client.c_str());


    // Service Call to waypoints conversion
    ros::ServiceClient client = nh.serviceClient<navigation::ConvertPixelWaypointsToMap>(convert_pixel_to_map_waypoints_srv_client);

    // Publisher for the converted waypoints as a path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(map_waypoint_path_topic_pub_, 10);

    nav_msgs::Path path;
    path.header.frame_id = base_frame; // Frame of RVIZ  

    ros::Rate loop_rate(1);  // 1 Hz

    while (ros::ok()) {
        // Prepare the service request
        navigation::ConvertPixelWaypointsToMap srv;
        srv.request.waypoint_file = waypoints_file;
        
        // Call service that converts waypoints to map waypoints
        if(client.call(srv)){
            if (srv.response.map_waypoints.empty()) {
                ROS_ERROR("No map waypoints were loaded after conversion.");
            } else {
                // Clear previous poses
                path.poses.clear();

                // Fill the nav_msgs/Path message with the converted waypoints
                for (const auto& waypoint : srv.response.map_waypoints) {
                    geometry_msgs::PoseStamped pose;
                    pose.header.frame_id = base_frame;
                    pose.header.stamp = ros::Time::now();

                    // Assigning x, y, and z coordinates from the converted waypoints
                    pose.pose.position.x = waypoint.x;
                    pose.pose.position.y = waypoint.y;
                    pose.pose.position.z = 0;  // Assuming a 2D plane

                    // No specific orientation is required, so keep the yaw as 0
                    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

                    path.poses.push_back(pose);
                }
                
                // Publish the path to RViz
                path.header.stamp = ros::Time::now();
                path_pub.publish(path);
                ROS_INFO("Published map waypoints as path.");
            }
        } else {
             ROS_ERROR("Failed to call service convert_pixel_waypoints_to_map_waypoints.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
