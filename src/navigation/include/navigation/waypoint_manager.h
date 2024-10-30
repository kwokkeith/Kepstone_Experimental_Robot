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
    bool initiateCoveragePath(navigation::InitiateCoveragePath::Request& req,
                              navigation::InitiateCoveragePath::Response& res);
    void publishNextWaypoint();
    bool updateWaypointStatus(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool getWaypoints(navigation::GetWaypoints::Request& req,
                      navigation::GetWaypoints::Response& res);

private:
    ros::NodeHandle nh_;
    ros::Publisher waypoint_pub_;
    ros::ServiceServer update_waypoint_service_;
    ros::ServiceServer initiate_coverage_service_;
    ros::ServiceServer get_waypoints_service_;

    std::vector<geometry_msgs::Point> waypoints_;
    size_t current_index_;
};
#endif