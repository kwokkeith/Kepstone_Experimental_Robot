import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from enum import Enum
from std_srvs.srv import SetBool
from navigation.srv import InitiateCoveragePath
from litter_destruction.srv import GlobalBoundaryCenter

# Enum for robot modes
class RobotMode(Enum):
    IDLE = 1
    COVERAGE = 2
    LITTER_PICKING = 3


class RobotController:
    def __init__(self):
        self.litter_picking_waypoint_topic = '/litter_picking/waypoint'
        self.coverage_path_waypoint_topic  = '/coverage_path/next_waypoint'

        ## Publishers
        # Publisher to control mux input selection
        self.mux_select_pub = rospy.Publisher('/waypoint_mux/select', String, queue_size=10)

        ## Subscribers
        # Subscriptions to the waypoint topics for each mode
        rospy.Subscriber(self.litter_picking_waypoint_topic, Point, self.litter_waypoint_callback)
        rospy.Subscriber(self.coverage_path_waypoint_topic, Point, self.coverage_waypoint_callback)

        ## Service Clients
        # Service to update waypoint_manager (get next waypoint)
        rospy.wait_for_service('/waypoint_manager/update_waypoint_status')
        self.update_waypoint_status = rospy.ServiceProxy('/waypoint_manager/update_waypoint_status', SetBool)

        # Service to initate the coverage path
        rospy.wait_for_service('/waypoint_manager/initiate_coverage_path')
        self.initiate_coverage_path = rospy.ServiceProxy('/waypoint_manager/initiate_coverage_path', InitiateCoveragePath)        
        
        # Service to get global boundary center
        rospy.wait_for_service('/litter_manager/get_global_boundary_center')
        self.get_global_boundary_center = rospy.ServiceProxy('/litter_manager/get_global_boundary_center', GlobalBoundaryCenter)

        ## Service Servers

        ## Configurations
        # ROS action client for move_base
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        # State and configuration variables
        self.mode = RobotMode.IDLE
        self.global_boundary_center = Point()
        rospy.loginfo("RobotController initialized and waiting for waypoints.")


    def initiate_coverage_mode(self, waypoints_file_path):
        """Initialize coverage mode by calling the initiate_coverage_path service with the waypoints file path."""
        try:
            # Prepare and send the request to the initiate_coverage_path service
            request = InitiateCoveragePath()
            request.waypoints_file = waypoints_file_path  # Set the file path for the waypoints

            # Call the service and wait for the response
            response = self.initiate_coverage_path(request)

            if response.success:
                self.mode = RobotMode.COVERAGE
                rospy.loginfo("Coverage path initiated successfully.")
            else:
                rospy.logwarn(f"Failed to initiate coverage path: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to initiate_coverage_path failed: {e}")


    def switch_mode(self, mode):
        """Switch the robot's mode and update the mux topic selection."""
        self.mode = mode
        
        if mode == RobotMode.IDLE:
            topic_name = '' 
        elif mode == RobotMode.COVERAGE:
            topic_name = self.coverage_path_waypoint_topic 
        elif mode == RobotMode.LITTER_PICKING:
            topic_name = self.litter_picking_waypoint_topic 
        else:
            rospy.WARN("In switch_mode, `mode` argument provided invalid")
            return
        
        self.mux_select_pub.publish(String(topic_name))
        rospy.loginfo(f"Switched to {mode.name} mode, using topic: {topic_name}")


    def handle_waypoint(self, waypoint):
        """Send a waypoint goal to move_base."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position = waypoint
        goal.target_pose.pose.orientation.w = 1.0  # Facing forward

        rospy.loginfo(f"Sending waypoint to move_base: {waypoint}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == SimpleActionClient.SimpleGoalState.SUCCEEDED:
            rospy.loginfo(f"Reached waypoint: {waypoint}")
            return True
        else:
            rospy.logwarn(f"Failed to reach waypoint: {waypoint}")
            return False


    def handle_litter_detection(self):
        """Switch to litter picking mode, clear litter, then return to coverage mode."""
        if not self.global_boundary_center:
            response = self.get_global_boundary_center(GlobalBoundaryCenter())
            if response.valid:
                self.global_boundary_center = response.center
            else:
                rospy.logwarn(f"Failed to get global boundary center")
                return

        # Switch to litter picking mode
        self.switch_mode(RobotMode.LITTER_PICKING)
        while not rospy.is_shutdown() and self.litter_manager.has_litter_to_clear():
            # Process litter waypoints until cleared (assuming litter_manager handles publishing waypoints)
            rospy.sleep(0.1)

        # Return to coverage mode and publish the last incomplete waypoint
        self.handle_waypoint(self.global_boundary_center)
        self.switch_mode(RobotMode.COVERAGE)


    def coverage_waypoint_callback(self, waypoint):
        """Coverage waypoints callback for coverage mode."""
        if self.mode == RobotMode.COVERAGE:
            success = self.handle_waypoint(waypoint)
            if success:
                response = self.update_waypoint_status(data=True)
                if response.sucess:
                    rospy.loginfo(f"Service call successful: {response.message}")
                else:
                    rospy.logwarn(f"Service call failed: {response.message}")
                

    def litter_waypoint_callback(self, waypoint):
        """Litter waypoints callback for litter picking mode."""
        if self.mode == RobotMode.LITTER_PICKING:
            self.handle_waypoint(waypoint)


def main():
    rospy.init_node('robot_controller_node')
    controller = RobotController()
    rospy.spin()

if __name__ == '__main__':
    main()