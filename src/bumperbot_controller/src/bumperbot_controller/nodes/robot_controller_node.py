import rospy
from geometry_msgs.msg import Point
from enum import Enum
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int32
from navigation.srv import InitiateCoveragePath, InitiateCoveragePathRequest, InitiateCoveragePathResponse
from litter_destruction.srv import GlobalBoundaryCenter, GlobalBoundaryCenterResponse
from bumperbot_controller.srv import ModeSwitch, ModeSwitchResponse
from bumperbot_controller.srv import GetCurrentMode, GetCurrentModeResponse


# Enum for robot modes
class RobotMode(Enum):
    IDLE               = 1
    COVERAGE           = 2
    LITTER_PICKING     = 3
    TRANSITION         = 4
    LITTER_TRACKING    = 5
    IDLE_LATCH         = 6

class RobotController:
    def __init__(self):
        ## Configurations
        self.mode = RobotMode.IDLE
        self.global_boundary_center = Point()
        self.global_boundary_radius = 0

        ## Get global parameters for topics/services
        # Service Servers
        mode_switch_srv = rospy.get_param('/robot_controller/services/mode_switch')
        get_global_boundary_srv = rospy.get_param('/robot_controller/services/get_global_boundary')
        get_current_mode_srv = rospy.get_param('/robot_controller/services/get_current_mode')
        initiate_coverage_srv = rospy.get_param('/robot_controller/services/initiate_coverage')
        stop_robot_job_srv = rospy.get_param('/robot_controller/services/stop_robot_job')
        start_robot_job_srv = rospy.get_param('/robot_controller/services/start_robot_job')
        # Service Clients
        clear_previous_job_client = rospy.get_param('/litter_manager/services/clear_previous_job')
        get_global_boundary_client = rospy.get_param('/litter_manager/services/get_global_boundary_center')
        initiate_coverage_path_client = rospy.get_param('/waypoint_manager/services/initiate_coverage_path')
        republish_global_boundary_client = rospy.get_param('/boundary_visualizer/services/republish_global_boundary')
        move_manager_client = rospy.get_param('/move_manager/services/cancel_all_goals')
        
        # Topic Publishers
        robot_mode_topic_pub = rospy.get_param('/robot_controller/topics/robot_mode')

        ## Publishers
        self.mode_pub = rospy.Publisher(robot_mode_topic_pub, Int32, queue_size=10)

        ## Subscribers
        # Subscriptions to the waypoint topics for each mode

        # Services servers to initialise first for controllers to start (Dependencies caused by wait_for_service)
        rospy.Service(mode_switch_srv, ModeSwitch, self.handle_mode_switch)                                         # Service server to request mode switch
        rospy.Service(get_global_boundary_srv, GlobalBoundaryCenter, self.handle_get_global_boundary_center)        # Service server to get global boundary center of robot controller
        rospy.Service(get_current_mode_srv, GetCurrentMode, self.handle_get_current_mode)                           # Service server to request current mode
        rospy.Service(stop_robot_job_srv, Trigger, self.handle_stop_robot_job)                                      # Service server to stop robot's current jobs
        rospy.Service(start_robot_job_srv, Trigger, self.handle_start_robot_job)                                    # Service server to initialise robot to running state

        ## Service Clients
        rospy.wait_for_service(initiate_coverage_path_client)
        rospy.wait_for_service(get_global_boundary_client)
        rospy.wait_for_service(republish_global_boundary_client)
        rospy.wait_for_service(clear_previous_job_client)
        rospy.wait_for_service(move_manager_client)

        self.initiate_coverage_path = rospy.ServiceProxy(initiate_coverage_path_client, InitiateCoveragePath)                     # Service to initate the coverage path   
        self.get_global_boundary_center_service = rospy.ServiceProxy(get_global_boundary_client, GlobalBoundaryCenter)            # Service to get global boundary center
        self.republish_global_boundary = rospy.ServiceProxy(republish_global_boundary_client, Trigger)                            # Service to republish global boundary center marker
        self.clear_previous_litter_job = rospy.ServiceProxy(clear_previous_job_client, Trigger)                                   # Service to stop and clear the previous litter job
        self.cancel_all_goals          = rospy.ServiceProxy(move_manager_client, Trigger)

        ## Service Servers
        rospy.Service(initiate_coverage_srv, InitiateCoveragePath, self.handle_initiate_coverage)       # Service server to initiate coverage


    def publish_mode(self):
        """Publishes the current robot mode."""
        mode_message = Int32()
        mode_message.data = self.mode.value  # Convert RobotMode enum to integer
        self.mode_pub.publish(mode_message)
        rospy.loginfo(f"Published robot mode: {self.mode.name}")


    def handle_initiate_coverage(self, req):
        """Service handler to initiate coverage path"""
        response = InitiateCoveragePathResponse()
        try:
            # Stop the previous litter picking job
            clear_job_response = self.clear_previous_litter_job()

            # Check if successful in stopping previous litter picking job
            if clear_job_response.success:
                rospy.loginfo(f"Successfully cleared the previous job: {clear_job_response.message}")
            else:
                rospy.logwarn(f"Failed to clear the previous job: {clear_job_response.message}")
                response.success = False
                response.message = "Failed to clear previous job."
                return response
            
            # Cancel all existing move goals
            cancel_all_goals_response = self.cancel_all_goals()
            # Check if successful in canceling all goals
            if cancel_all_goals_response.success:
                rospy.loginfo("Successfully canceled all goals.")
            else:
                rospy.logwarn("Failed to cancel all goals.")
                response.success = False
                response.message = "Failed to cancel existing move goals."
                return response

            # Set the waypoints file path
            request = InitiateCoveragePathRequest(waypoints_file=req.waypoints_file)

            # Call the service to initiate coverage
            service_response = self.initiate_coverage_path(request)

            if service_response.success:
                rospy.loginfo("Coverage path initiated successfully.")
                response.success = True
                response.message = "Coverage path successfully initiated."

                # Set mode to COVERAGE if initiation was successful
                self.switch_mode(RobotMode.COVERAGE)
            else:
                rospy.logwarn(f"Coverage path initiation failed: {service_response.message}")
                response.success = False
                response.message = f"Failed to initiate coverage path: {service_response.message}"

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to initiate coverage path failed: {e}")
            response.success = False
            response.message = f"Service call failed: {e}"

        return response
    

    def handle_get_global_boundary_center(self, req):
        """Service handle to get global boundary center"""
        response = GlobalBoundaryCenterResponse()
        if self.global_boundary_center:
            response.center = self.global_boundary_center
            response.radius = self.global_boundary_radius
            response.valid = True
        else:
            response.center = Point()
            response.valid = False
        return response


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
            else:
                raise ValueError("Invalid mode requested.")
        except Exception as e:
            response.success = False
            response.message = str(e)
            rospy.logwarn(response.message) 
        return response
    

    def handle_stop_robot_job(self, req):
        """Service handler to stop robot's current job"""
        try:
            self.switch_mode(RobotMode.IDLE_LATCH)
        except Exception as e:
            success = False
            message = "Error stopping robot because of failed Robot mode switch"
            return TriggerResponse(success=success, message=message)
        try:
            clear_job_response = self.clear_previous_litter_job()
            if not clear_job_response.success:
                success = False
                message = "Error stopping robot's previous litter task"
                return TriggerResponse(success=success, message=message)
        except Exception as e:
            success = False
            message = "Error stopping robot's previous litter task"
            return TriggerResponse(success=success, message=message)

        success = True
        message = "Succeeded in stopping robot job"
        return TriggerResponse(success=success, message=message)


    def handle_start_robot_job(self, req):
        """Service handler to start robot, get it running. Allow it to move if it detects litter"""
        try:
            self.switch_mode(RobotMode.IDLE)
        except Exception as e:
            success = False
            message = "Error stopping robot because of failed Robot mode switch"
            return TriggerResponse(success=success, message=message)
        success = True
        message = "Robot is now running and can detect and move."
        return TriggerResponse(success=success, message=message)


    def switch_mode(self, mode):
        """Switch the robot's mode and update the mux topic selection."""
        #######################################
        # PRE-CONDITIONS before switching modes
        #######################################
        # Check if robot is in transitioning mode to the center when asked to do litter picking (LITTER_TO_COVERAGE mode)
        if mode == RobotMode.LITTER_PICKING:
            if not self.mode == RobotMode.TRANSITION:
                # Still in transition, then if we detect litter then we shall clear it but dont change global center
                # O.w then we change the global center
                # Set global center
                if not self.get_global_boundary_center():
                    rospy.logerr("Failed to retrieve global boundary's center while changing to LITTER_PICKING mode")

        ######################
        # MODE switching logic
        ######################
        # Set new mode
        self.mode = mode

        # Trigger the global boundary visualization
        response = self.republish_global_boundary()

        self.publish_mode()      # Publish new mode to the topic   
        # Determining topic based on mode and start the appropriate mode function
        if mode == RobotMode.IDLE:
            # Cancel all existing move goals
            cancel_all_goals_response = self.cancel_all_goals()
            # Check if successful in canceling all goals
            if cancel_all_goals_response.success:
                rospy.loginfo("Successfully canceled all goals.")
            else:
                rospy.logwarn("Failed to cancel all goals.")
                response.success = False
                response.message = "Failed to cancel existing move goals."
                return response
            rospy.loginfo("Switched to IDLE mode.")

        elif mode == RobotMode.COVERAGE:
            rospy.loginfo("Switched to COVERAGE mode.")

        elif mode == RobotMode.LITTER_PICKING:
            if response.success:
                rospy.loginfo("Global boundary marker published.")
            else:
                rospy.logwarn(f"Failed to publish global boundary marker: {response.message}")        
            rospy.loginfo("Switched to LITTER_PICKING mode.")

        elif mode == RobotMode.TRANSITION:
            rospy.loginfo("Switched to TRANSITION mode")

        elif mode == RobotMode.LITTER_TRACKING:
            # Cancel all existing move goals
            cancel_all_goals_response = self.cancel_all_goals()
            # Check if successful in canceling all goals
            if cancel_all_goals_response.success:
                rospy.loginfo("Successfully canceled all goals.")
            else:
                rospy.logwarn("Failed to cancel all goals.")
                response.success = False
                response.message = "Failed to cancel existing move goals."
                return response
            
            rospy.loginfo("Switched to LITTER_TRACKING mode")

        else:
            rospy.logwarn("In switch_mode, `mode` argument provided invalid")
    

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
                self.switch_mode(RobotMode.COVERAGE)
                rospy.loginfo("Coverage path initiated successfully.")
            else:
                rospy.logwarn(f"Failed to initiate coverage path: {response.message}")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to initiate_coverage_path failed: {e}")


    def get_global_boundary_center(self):
        """To get the global boundary center using the service provided by litter_manager"""
        response = self.get_global_boundary_center_service()
        if response.valid:
            self.global_boundary_center = response.center
            self.global_boundary_radius = response.radius
            return True
        return False

def main():
    rospy.init_node('robot_controller_node')
    controller = RobotController()
    rospy.spin()

if __name__ == '__main__':
    main()