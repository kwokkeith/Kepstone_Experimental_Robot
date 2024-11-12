#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

class WaypointNav:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('waypoint_nav')

        # Create an action client to send goals to the move_base server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Define an array of waypoints
        self.waypoints = [
            Pose(Point(1.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(2.0, 1.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(2.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(Point(1.0, 2.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        ]

        # Start the waypoint navigation
        self.navigate_waypoints()

    def navigate_waypoints(self):
        for i, waypoint in enumerate(self.waypoints):
            # Create a goal for move_base
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = waypoint

            # Send the goal to the move_base action server
            rospy.loginfo(f"Sending waypoint {i+1}")
            self.client.send_goal(goal)

            # Wait for the robot to reach the goal
            self.client.wait_for_result()

            # Check if the robot succeeded in reaching the goal
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached waypoint {i+1}")
            else:
                rospy.logwarn(f"Failed to reach waypoint {i+1}")

        rospy.loginfo("All waypoints reached")

if __name__ == "__main__":
    try:
        WaypointNav()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted")