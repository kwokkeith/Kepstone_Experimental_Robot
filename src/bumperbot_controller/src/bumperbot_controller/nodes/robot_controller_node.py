import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from enum import Enum
from bumperbot_detection.msg import LitterPoint
from std_srvs.srv import SetBool
from navigation.srv import InitiateCoveragePath, InitiateCoveragePathRequest
from litter_destruction.srv import GlobalBoundaryCenter
from litter_destruction.srv import GetNextLitter
from litter_destruction.srv import RemoveLitter, RemoveLitterRequest 
from litter_destruction.srv import HasLitterToClear
from bumperbot_controller.srv import ModeSwitch, ModeSwitchResponse
from bumperbot_controller.srv import GetCurrentMode, GetCurrentModeResponse

# Enum for robot modes
class RobotMode(Enum):
    IDLE = 1
    COVERAGE = 2
    LITTER_PICKING = 3


class RobotController:
    def __init__(self):
        self.litter_picking_waypoint_topic = '/litter_manager/next_waypoint'
        self.coverage_path_waypoint_topic  = '/coverage_path/next_waypoint'

        ## Publishers
        self.mux_select_pub = rospy.Publisher('/robot_controller/waypoint_mux/select', String, queue_size=10)         # Publisher to control mux input selection

        ## Subscribers
        # Subscriptions to the waypoint topics for each mode
        rospy.Subscriber(self.litter_picking_waypoint_topic, LitterPoint, self.litter_waypoint_callback)
        rospy.Subscriber(self.coverage_path_waypoint_topic, Point, self.coverage_waypoint_callback)

        ## Service Clients
        rospy.wait_for_service('/waypoint_manager/get_next_waypoint')
        rospy.wait_for_service('/waypoint_manager/initiate_coverage_path')
        rospy.wait_for_service('/litter_manager/get_global_boundary_center')
        rospy.wait_for_service('/litter_manager/get_next_litter')
        rospy.wait_for_service('/litter_manager/delete_litter')
        rospy.wait_for_service('/litter_manager/has_litter_to_clear')

        self.update_waypoint_status = rospy.ServiceProxy('/waypoint_manager/get_next_waypoint', SetBool)                             # Service to update waypoint_manager (get next waypoint)
        self.initiate_coverage_path = rospy.ServiceProxy('/waypoint_manager/initiate_coverage_path', InitiateCoveragePath)           # Service to initate the coverage path   
        self.get_global_boundary_center = rospy.ServiceProxy('/litter_manager/get_global_boundary_center', GlobalBoundaryCenter)     # Service to get global boundary center
        self.get_next_litter = rospy.ServiceProxy('/litter_manager/get_next_litter', GetNextLitter)                                  # Service to update to next litter in litter manager 
        self.delete_litter   = rospy.ServiceProxy('/litter_manager/delete_litter', RemoveLitter)                                     # Service to delete litter in litter manager
        self.has_litter_to_clear = rospy.ServiceProxy('/litter_manager/has_litter_to_clear', HasLitterToClear)                       # Service to check if any more litter to clear (litter manager)

        ## Service Servers
        rospy.Service('/robot_controller/mode_switch', ModeSwitch, self.handle_mode_switch)                     # Service server to request mode switch
        rospy.Service('/robot_controller/get_current_mode', GetCurrentMode, self.handle_get_current_mode)       # Service server to request current mode

        ## Configurations
        # ROS action client for move_base
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


        self.mode = RobotMode.IDLE
        self.global_boundary_center = Point()
        rospy.loginfo("RobotController initialized and waiting for waypoints.")


    def handle_get_current_mode(self, req):
        """Service handler to get current mode of robot controller"""
        response = GetCurrentModeResponse()
        try:
            response.mode = self.mode.value
            response.success = True
        except:
            response.mode = -1
            response.success = False
        return response


    def handle_mode_switch(self, req):
        """Service handler to switch mode (RobotMode enum)"""
        response = ModeSwitchResponse()
        try:
            # Map the integer request to a RobotMode, raising an error if invalid
            if req.mode in RobotMode._value2member_map_:     # Check if in RobotMode
                self.switch_mode(RobotMode(req.mode))
                response.success = True
                response.message = f"Switched to {RobotMode(req.mode).name} mode."
                rospy.loginfo(response.message)
            else:
                raise ValueError("Invalid mode requested.")
        except Exception as e:
            response.success = False
            response.message = str(e)
            rospy.logwarn(response.message) 
        return response
    

    def switch_mode(self, mode):
        """Switch the robot's mode and update the mux topic selection."""
        self.mode = mode
        
        if mode == RobotMode.IDLE:
            topic_name = '' 
        elif mode == RobotMode.COVERAGE:
            topic_name = self.coverage_path_waypoint_topic
            self.perform_coverage_mode() 
        elif mode == RobotMode.LITTER_PICKING:
            topic_name = self.litter_picking_waypoint_topic 
            self.perform_litter_mode()
        else:
            rospy.WARN("In switch_mode, `mode` argument provided invalid")
            return
        
        self.mux_select_pub.publish(String(topic_name))
        rospy.loginfo(f"Switched to {mode.name} mode, using next waypoint topic: {topic_name}")


    def initiate_coverage_mode(self, waypoints_file_path):
        """Initialize coverage mode by calling the initiate_coverage_path service with the waypoints file path."""
        try:
            # Prepare and send the request to the initiate_coverage_path service
            request = InitiateCoveragePathRequest()
            request.waypoints_file = waypoints_file_path  # Set the file path for the waypoints

            # Call the service and wait for the response
            response = self.initiate_coverage_path(request)

            if response.success:
                # Successfully initiated coverage path, set the robot into COVERAGE mode
                self.mode = RobotMode.COVERAGE
                rospy.loginfo("Coverage path initiated successfully.")
            else:
                rospy.logwarn(f"Failed to initiate coverage path: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to initiate_coverage_path failed: {e}")
   

    def navigate_to_waypoint(self, waypoint):
        """Send a waypoint goal to move_base."""
        # Ensure connection to move_base action server
        # Wait for the server to be available
        if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn("move_base action server is not available.")
            return False

        # Prepare goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.point.x
        goal.target_pose.pose.position.y = waypoint.point.y
        goal.target_pose.pose.position.z = 0.0

        # Set Orientation
        goal.target_pose.pose.orientation.w = 1.0       # No rotation (facing forward)

        rospy.loginfo(f"Sending waypoint to move_base: {waypoint}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == SimpleActionClient.SimpleGoalState.SUCCEEDED:
            rospy.loginfo(f"Reached waypoint: {waypoint}")
            return True
        else:
            rospy.logwarn(f"Failed to reach waypoint: {waypoint}")
            return False


    def perform_litter_mode(self):
        """Performs the litter clearing algorithm in LITTER_PICKING mode."""
        rospy.loginfo("Performing litter mode...")
        response = self.get_next_litter()
        if not response.success:
            rospy.WARN("Failed to call service /litter_manager/get_next_litter in perform_litter_mode")
            return      # Skip the rest
        while not rospy.is_shutdown() and self.mode == RobotMode.LITTER_PICKING:
            # Get the next litter waypoint
            try:
                litter_point = rospy.wait_for_message(self.litter_picking_waypoint_topic, LitterPoint, timeout=5)
            except rospy.ROSException as e:
                rospy.logwarn(f"Failed to get litter waypoint from topic: {e}")
                break

            rospy.loginfo(f"Navigating to \n{litter_point.point}")
            # Navigate to the litter waypoint
            if self.navigate_to_waypoint(litter_point.point):
                 # Wait until the robot reaches the waypoint before proceeding
                rospy.loginfo(f"Reached litter waypoint at {litter_point.point}, preparing for litter destruction.")
                
                # Simulate the litter destruction process
                rospy.loginfo(f"Destroying litter at {litter_point.point}")
                rospy.sleep(5)  # Simulate time taken to destroy litter

                # Remove the litter from the memory and manager
                try:
                    delete_request = RemoveLitterRequest(litter_point=litter_point)
                    delete_response = self.delete_litter(delete_request)
                    
                    if delete_response.success:
                        rospy.loginfo(f"Litter with ID {litter_point.id} successfully deleted.")
                    else:
                        rospy.logwarn(f"Failed to delete litter with ID {litter_point.id}.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call to delete litter failed: {e}")
                    continue

                # Check if there is more litter to clear
                try:
                    response = self.has_litter_to_clear()
                    
                    if not response.has_litter:
                        rospy.loginfo("All litter cleared. Switching back to coverage mode.")
                        self.switch_mode(RobotMode.COVERAGE)
                        self.perform_coverage_mode()  # Assume this is implemented elsewhere
                        break                         # Exit Litter Picking Mode
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call to has_litter_to_clear failed: {e}")
                    break

                # Get the next litter waypoint if more litter exists
                try:
                    response = self.get_next_litter()
                    
                    if not response.success:
                        rospy.logwarn("Failed to get next litter.")
                        break
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call to get_next_litter failed: {e}")
                    break
            else:
                rospy.logwarn("Failed to reach the litter waypoint.")


    def connect_to_move_base(self):
        """Establish or re-establish connection with the move_base action server."""
        rospy.loginfo("Connecting to move_base action server...")
        if not self.move_base_client.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("Failed to connect to move_base. Retrying...")
            return False
        rospy.loginfo("Connected to move_base action server.")
        return True


    def perform_coverage_mode(self):
        pass


    def handle_litter_detection(self):
        while not rospy.is_shutdown() and self.litter_manager.has_litter_to_clear():
            # Process litter waypoints until cleared (assuming litter_manager handles publishing waypoints)
            rospy.sleep(0.1)

        # Return to coverage mode and publish the last incomplete waypoint
        self.navigate_to_waypoint(self.global_boundary_center)
        self.switch_mode(RobotMode.COVERAGE)


    def coverage_waypoint_callback(self, waypoint):
        """Coverage waypoints callback for coverage mode."""
        if self.mode == RobotMode.COVERAGE:
            success = self.navigate_to_waypoint(waypoint)
            if success:
                response = self.update_waypoint_status(data=True)
                if response.sucess:
                    rospy.loginfo(f"Service call successful: {response.message}")
                else:
                    rospy.logwarn(f"Service call failed: {response.message}")
                

    def litter_waypoint_callback(self, waypoint):
        """Litter waypoints callback for litter picking mode."""
        if self.mode == RobotMode.LITTER_PICKING:
            self.navigate_to_waypoint(waypoint)


def main():
    rospy.init_node('robot_controller_node')
    controller = RobotController()
    rospy.spin()

if __name__ == '__main__':
    main()