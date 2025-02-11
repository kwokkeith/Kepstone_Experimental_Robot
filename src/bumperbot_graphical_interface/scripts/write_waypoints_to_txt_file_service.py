#!/usr/bin/env python3

import rospy
from bumperbot_graphical_interface.srv import WriteWaypoints, WriteWaypointsResponse
from std_msgs.msg import String
#THIS PYTHON SERVICE NEEDS TO HAVE EXECUTABLE PERMISSIONS
#chmod +x write_waypoints_to_txt_file_service.py

def handle_write_waypoints(req):
    file_path = rospy.get_param("~waypoints_file_path")
    # rospy.loginfo("Received waypoints list: %s", req.waypoints_list)
    
    try:
        with open(file_path, 'w') as f:
            f.write(req.waypoints_list)
        rospy.loginfo("Waypoints written to file: %s", file_path)
        return WriteWaypointsResponse(success=True, message="Waypoints successfully written to file.")
    except Exception as e:
        rospy.logerr("Failed to write waypoints to file: %s", e)
        return WriteWaypointsResponse(success=False, message=str(e))

def main():
    rospy.init_node('write_waypoints_to_txt_file_service')
    service = rospy.Service('write_waypoints_to_txt_file', WriteWaypoints, handle_write_waypoints)
    rospy.loginfo("Service [write_waypoints_to_txt_file] is ready.")
    rospy.spin()

if __name__ == '__main__':
    main()