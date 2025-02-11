#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from navigation.srv import InitiateCoveragePath, InitiateCoveragePathRequest

def initiate_coverage_callback(req):
    waypoints_file_path = rospy.get_param('~waypoints_file_path')
    try:
        rospy.wait_for_service('/robot_controller/initiate_coverage',timeout=4)
        initiate_coverage = rospy.ServiceProxy('/robot_controller/initiate_coverage', InitiateCoveragePath)
        request = InitiateCoveragePathRequest(waypoints_file=waypoints_file_path)
        response = initiate_coverage(request)
        if response.success:
            rospy.loginfo("Coverage path initiated successfully.")
            return TriggerResponse(success=True, message="Coverage path initiated successfully.")
        else:
            rospy.logwarn(f"Failed to initiate coverage path: {response.message}")
            return TriggerResponse(success=False, message=f"Failed to initiate coverage path: {response.message}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return TriggerResponse(success=False, message=f"Service call failed: {e}")

def main():
    rospy.init_node('start_coverage_service')
    service = rospy.Service('/start_coverage', Trigger, initiate_coverage_callback)
    rospy.loginfo("Ready to start coverage service when triggered by /start_coverage service.")
    rospy.spin()

if __name__ == "__main__":
    main()