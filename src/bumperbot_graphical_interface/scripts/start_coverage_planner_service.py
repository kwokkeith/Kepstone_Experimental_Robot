#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess

def handle_start_coverage_planner(req):
    try:
        # Start the launch file using a subprocess
        subprocess.Popen(['roslaunch', 'coverage_planner', 'coverage_planner.launch'])
        return TriggerResponse(success=True, message="Launch file started successfully")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    rospy.init_node('start_coverage_planner_service')
    service = rospy.Service('start_coverage_planner', Trigger, handle_start_coverage_planner)
    rospy.loginfo("Service /start_coverage_planner is ready.")
    rospy.spin()