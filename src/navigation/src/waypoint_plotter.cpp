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

    // Service Call to waypoints conversion
    ros::ServiceClient client = nh.serviceClient<navigation::ConvertPixelWaypointsToMap>("convert_pixel_waypoints_to_map_waypoints");

    // Publisher for the converted waypoints as a path
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("map_waypoint_path", 10);

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
