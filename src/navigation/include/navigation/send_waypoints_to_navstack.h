#ifndef SEND_WAYPOINTS_TO_NAVSTACK_H
#define SEND_WAYPOINTS_TO_NAVSTACK_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <fstream>

// Define the MoveBaseActionClient type
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Function sends a goal to move_base
bool sendGoal(const geometry_msgs::Point& waypoint, MoveBaseClient &ac);

// Function sends waypoints to the navigation stack
void sendWaypointsToNavStack(const std::vector<geometry_msgs::Point> &waypoints, MoveBaseClient &ac);

#endif
