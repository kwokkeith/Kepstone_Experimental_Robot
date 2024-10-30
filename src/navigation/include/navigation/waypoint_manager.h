#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/SetBool.h>
#include "navigation/InitiateCoveragePath.h"
#include "navigation/GetWaypoints.h"

class WaypointManager {
public:
    WaypointManager(ros::NodeHandle& nh);

private:
   // ROS node handle
    ros::NodeHandle nh_;

    // ROS publishers and services
    ros::Publisher waypoint_pub_;
    ros::ServiceServer update_waypoint_service_;
    ros::ServiceServer initiate_coverage_service_;
    ros::ServiceServer get_waypoints_service_;

    // Timer for continuous waypoint publication
    ros::Timer waypoint_timer_;

    // Waypoints storage and index tracking
    std::vector<geometry_msgs::Point> waypoints_;
    size_t current_index_;

    // Helper methods
    bool initiateCoveragePath(navigation::InitiateCoveragePath::Request& req,
                              navigation::InitiateCoveragePath::Response& res);
    bool getNextWaypoint(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool getWaypoints(navigation::GetWaypoints::Request& req, navigation::GetWaypoints::Response& res);

    void publishNextWaypoint();               // Publishes the next waypoint in sequence
    void timerCallback(const ros::TimerEvent&); // Timer callback for continuous waypoint publishing
};
#endif