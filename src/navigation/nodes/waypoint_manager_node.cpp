#include <ros/ros.h>
#include "navigation/waypoint_manager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_manager_node");
    ros::NodeHandle nh;

    // Initialize WaypointManager, which now handles both waypoint initiation and updates
    WaypointManager waypoint_manager(nh);

    ROS_INFO("WaypointManager is ready to initiate coverage path and update waypoints.");

    ros::spin();  // Keep node running to handle service requests
    return 0;
}