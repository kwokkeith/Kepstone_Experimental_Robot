#include "navigation/send_waypoints_to_navstack.h"

// Function: Sends a goal to move_base (ROS Navstack navigation)
void sendGoal(const geometry_msgs::Point& waypoint, MoveBaseClient &ac) {
    // Define goal message
    move_base_msgs::MoveBaseGoal goal;

    // Set the goal position and orientation
    goal.target_pose.header.frame_id = "map"; // Goal should be in "map" frame
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint.x;
    goal.target_pose.pose.position.y = waypoint.y;
    goal.target_pose.pose.position.z = 0.0;

    // Set orientation
    goal.target_pose.pose.orientation.w = 1.0; // No rotation (facing forward)

    ROS_INFO("Sending goal: (%.2f, %.2f)", waypoint.x, waypoint.y);
    
    // Send the goal and wait for the robot to reach the waypoint
    ac.sendGoal(goal);

    // Wait for the action to return
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Successfully reached waypoint (%.2f, %.2f)", waypoint.x, waypoint.y);
    else
        ROS_WARN("Failed to reach waypoint (%.2f, %.2f)", waypoint.x, waypoint.y);
}


// Function: Send waypoints to ROS Navstack Nav package
void sendWaypointsToNavStack(const std::vector<geometry_msgs::Point> &waypoints, MoveBaseClient &ac) {
    // Loop through each waypoint and send it to move_base
    for (const auto& waypoint : waypoints) {
        sendGoal(waypoint, ac);
    }
}
