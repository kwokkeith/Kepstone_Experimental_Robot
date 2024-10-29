import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from enum import Enum
from litter_destruction.litter_manager import LitterManager
from navigation.waypoint_manager import WaypointManager
from std_srvs.srv import SetBool
from navigation.srv import InitiateCoveragePath

# Enum for robot modes
class RobotMode(Enum):
    COVERAGE = 1
    LITTER_PICKING = 2


class RobotController:
    def __init__(self, waypoint_manager, litter_manager):
        # Initialize managers
        self.waypoint_manager = waypoint_manager
        self.litter_manager = litter_manager

        # ROS action client for move_base
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        # Publisher to control mux input selection
        self.mux_select_pub = rospy.Publisher('/waypoint_mux/select', String, queue_size=10)

        # Service to update waypoint_manager (get next waypoint)
        rospy.wait_for_service('/waypoint_manager/update_waypoint_status')
        self.update_waypoint_status = rospy.ServiceProxy('/waypoint_manager/update_waypoint_status', SetBool)

        # Service to initate the coverage path
        rospy.wait_for_service('/waypoint_manager/initiate_coverage_path')
        self.initiate_coverage_path = rospy.ServiceProxy('/waypoint_manager/initiate_coverage_path', InitiateCoveragePath)        
        
        # State and configuration variables
        self.mode = RobotMode.COVERAGE
        self.global_boundary_center = Point()
        rospy.loginfo("RobotController initialized and waiting for waypoints.")

        # Subscriptions to the waypoint topics for each mode
        rospy.Subscriber('/litter_picking/waypoint', Point, self.litter_waypoint_callback)
        rospy.Subscriber('/coverage_path/next_waypoint', Point, self.coverage_waypoint_callback)


    def initiate_coverage_mode(self, waypoints_file_path):
        """Initialize coverage mode by calling the initiate_coverage_path service with the waypoints file path."""
        try:
            # Prepare and send the request to the initiate_coverage_path service
            request = InitiateCoveragePath()
            request.waypoints_file = waypoints_file_path  # Set the file path for the waypoints

            # Call the service and wait for the response
            response = self.initiate_coverage_path(request)

            if response.success:
                rospy.loginfo("Coverage path initiated successfully.")
            else:
                rospy.logwarn(f"Failed to initiate coverage path: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to initiate_coverage_path failed: {e}")


    def switch_mode(self, mode):
        """Switch the robot's mode and update the mux topic selection."""
        self.mode = mode
        topic_name = '/litter_picking/waypoint' if mode == RobotMode.LITTER_PICKING else '/coverage_path/next_waypoint'
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
            self.global_boundary_center = self.litter_manager.global_boundary_center

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

    # Load coverage waypoints as an example
    coverage_waypoints = [Point(x=1, y=1), Point(x=2, y=2), Point(x=3, y=3)]

    # Initialize managers and controller
    nh = rospy.NodeHandle()
    waypoint_manager = WaypointManager(nh)
    litter_manager = LitterManager(distance_threshold=2.0, min_local_radius=0.5, max_local_radius=1.5)
    controller = RobotController(waypoint_manager, litter_manager)

    controller.initiate_coverage_mode(coverage_waypoints)
    rospy.spin()

if __name__ == '__main__':
    main()
