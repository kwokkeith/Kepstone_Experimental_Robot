#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess

def handle_stop_coverage_planner(req):
    try:
        # Kill the nodes by name using rosnode kill
        subprocess.Popen(['rosnode', 'kill', 'coverage_planner_node'])
        subprocess.Popen(['rosnode', 'kill', 'db_publisher_node'])
        # rospy.loginfo("Requested termination for coverage_planner_node.")
        rospy.loginfo("Requested termination for coverage_planner_node and db_publisher_node.")
        return TriggerResponse(success=True, message="Nodes killed successfully.")
    except Exception as e:
        rospy.logerr("Failed to kill nodes: %s", str(e))
        return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    rospy.init_node('stop_coverage_planner_service')
    service = rospy.Service('stop_coverage_planner', Trigger, handle_stop_coverage_planner)
    rospy.loginfo("Service /stop_coverage_planner is ready.")
    rospy.spin()