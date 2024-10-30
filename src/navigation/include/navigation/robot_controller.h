#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/SetBool.h>
#include <navigation/InitiateCoveragePath.h>
#include "navigation/waypoint_manager.h"
#include <bumperbot_utils/robot_mode.h>

class RobotController {
public:
    RobotController(ros::NodeHandle& nh);

    void initiateCoverageMode(const std::string& waypoints_file_path);
    void switchMode(RobotMode mode);
    bool handleWaypoint(const geometry_msgs::Point& waypoint);
    void handleLitterDetection();

private:
    void coverageWaypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);
    void litterWaypointCallback(const geometry_msgs::Point::ConstPtr& waypoint);

    ros::NodeHandle& nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    ros::Publisher mux_select_pub_;
    ros::Subscriber litter_sub_;
    ros::Subscriber coverage_sub_;

    ros::ServiceClient update_waypoint_status_;
    ros::ServiceClient initiate_coverage_path_;

    RobotMode mode_;
    geometry_msgs::Point global_boundary_center_;
    bool global_boundary_center_set_ = false;
};

#endif 
