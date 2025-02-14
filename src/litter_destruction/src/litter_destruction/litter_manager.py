import rospy
import threading
import heapq
import math
from geometry_msgs.msg import Point, PoseStamped
from bumperbot_detection.msg import  LitterPoint, DetectedLitterPoint
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.srv import GetPlan, GetPlanRequest
from navigation.srv import GetAmclPose
from bumperbot_detection.srv import DeleteLitterRequest, DeleteLitter, GetLitterList
from litter_destruction.srv import GlobalBoundaryCenter, GlobalBoundaryCenterResponse
from litter_destruction.srv import LocalBoundaryCenter, LocalBoundaryCenterResponse
from litter_destruction.srv import GetLitterSet, GetLitterSetResponse
from litter_destruction.srv import GetNextLitter, GetNextLitterResponse
from litter_destruction.srv import RemoveLitter, RemoveLitterResponse
from litter_destruction.srv import HasLitterToClear, HasLitterToClearResponse
from litter_destruction.srv import GetNextTargetLitter, GetNextTargetLitterResponse
from bumperbot_controller.srv import ModeSwitch, ModeSwitchRequest
from bumperbot_controller.srv import GetCurrentMode

class LitterManager:
    def __init__(self, distance_threshold=5, min_local_radius=1, max_local_radius=3):
        ## Get global configurations
        # Service Clients
        get_litter_list_client = rospy.get_param('/litter_memory/services/get_litter_list')
        delete_litter_client = rospy.get_param('/litter_memory/services/delete_litter')
        clear_memory_client = rospy.get_param('/litter_memory/services/clear_memory')
        get_amcl_pose_client = rospy.get_param('/navigation/services/get_amcl_pose')
        mode_switch_client = rospy.get_param('/robot_controller/services/mode_switch')
        get_global_boundary_client = rospy.get_param('/robot_controller/services/get_global_boundary')
        get_current_mode_client = rospy.get_param('/robot_controller/services/get_current_mode')
        republish_global_boundary_client = rospy.get_param('/boundary_visualizer/services/republish_global_boundary')
        republish_local_boundary_client = rospy.get_param('/boundary_visualizer/services/republish_local_boundary')
        make_plan_client = rospy.get_param('/navigation/services/navfn_make_plan')
        # Service Servers
        get_global_boundary_center_srv = rospy.get_param('/litter_manager/services/get_global_boundary_center')
        get_local_boundary_center_srv = rospy.get_param('/litter_manager/services/get_local_boundary_center')
        get_litter_set_srv = rospy.get_param('/litter_manager/services/get_litter_set')
        get_next_litter_srv = rospy.get_param('/litter_manager/services/get_next_litter')
        has_litter_to_clear_srv = rospy.get_param('/litter_manager/services/has_litter_to_clear')
        delete_litter_srv = rospy.get_param('/litter_manager/services/delete_litter')
        next_waypoint_srv = rospy.get_param('/litter_manager/services/next_waypoint')
        clear_previous_job_srv = rospy.get_param('/litter_manager/services/clear_previous_job')
        # Topics Subscribers
        new_litter_sub = rospy.get_param('/litter_memory/topics/new_litter')


        ## Node Variables
        self.litter_mutex = threading.Lock()
        self.set_litter = set()
        self.min_heap_litter = []
        self.start_pos = Point()
        self.global_boundary_center = None            # Center of global boundary
        self.distance_threshold = distance_threshold  # Radius for the global boundary
        self.local_boundary_center = None             # Center of local boundary
        self.min_local_radius = min_local_radius      # Min radius of local boundary
        self.max_local_radius = max_local_radius      # Max radius of local boundary
        self.local_boundary_radius = 0                # Radius of local boundary
        self.next_litter = None                       # To store the next litter to continuously publish

        ## Publisher

        ## Subscribers
        rospy.Subscriber(new_litter_sub, DetectedLitterPoint, self.detection_callback)
        
        ## Service Clients
        rospy.wait_for_service(get_litter_list_client)
        rospy.wait_for_service(delete_litter_client)
        rospy.wait_for_service(get_amcl_pose_client)
        rospy.wait_for_service(mode_switch_client)
        rospy.wait_for_service(get_global_boundary_client)
        rospy.wait_for_service(republish_global_boundary_client)
        rospy.wait_for_service(republish_local_boundary_client)
        rospy.wait_for_service(get_current_mode_client)
        rospy.wait_for_service(clear_memory_client)
        rospy.wait_for_service(make_plan_client)         # Usage of Navfn topic of move_base allows for planning even with active plan

        self.get_litter_list_srv        = rospy.ServiceProxy(get_litter_list_client, GetLitterList)                       # Define the service client for getting litter list
        self.delete_litter_srv          = rospy.ServiceProxy(delete_litter_client, DeleteLitter)                          # Define the service client for deleting litter
        self.get_pose_srv               = rospy.ServiceProxy(get_amcl_pose_client, GetAmclPose)                           # Define service client for getting amcl pose
        self.req_mode                   = rospy.ServiceProxy(mode_switch_client, ModeSwitch)                              # Define service client for changing robot controller to litter mode
        self.get_global_boundary_center = rospy.ServiceProxy(get_global_boundary_client, GlobalBoundaryCenter)            # Define service client for getting global boundary centre from robot controller
        self.republish_global_boundary  = rospy.ServiceProxy(republish_global_boundary_client, Trigger)                   # Define service client to replot the global boundary RVIZ marker
        self.republish_local_boundary   = rospy.ServiceProxy(republish_local_boundary_client, Trigger)                    # Define service client to replot the local boundary RVIZ marker
        self.get_robot_mode             = rospy.ServiceProxy(get_current_mode_client, GetCurrentMode)                     # Define service client to get robot current mode
        self.clear_litter_memory_srv    = rospy.ServiceProxy(clear_memory_client, Trigger)                                # Define service client to clear litter memory
        self.make_plan_srv              = rospy.ServiceProxy(make_plan_client, GetPlan)                                   # Define service to make a plan to calculate distance to litter using NavStack goals

        ## Service Servers
        rospy.Service(get_global_boundary_center_srv, GlobalBoundaryCenter, self.handle_get_global_boundary_center)   # Define service server to get global boundary center
        rospy.Service(get_local_boundary_center_srv, LocalBoundaryCenter, self.handle_get_local_boundary_center)      # Define service server to get local boundary center
        rospy.Service(get_litter_set_srv, GetLitterSet, self.handle_get_litter_set)                                   # Define service server to get litter set
        rospy.Service(get_next_litter_srv, GetNextLitter, self.handle_get_next_litter)                                # Define service server to get next litter
        rospy.Service(has_litter_to_clear_srv, HasLitterToClear, self.handle_has_litter_to_clear)                     # Define service server to check if there is still litter in set
        rospy.Service(delete_litter_srv, RemoveLitter, self.handle_remove_litter)                                     # Define service server to delete litter from litter manager
        rospy.Service(next_waypoint_srv, GetNextTargetLitter, self.handle_get_next_target_litter)                     # Define service server to get the next target litter
        rospy.Service(clear_previous_job_srv, Trigger, self.handle_clear_previous_job)                                # Define service server to clear previous job


    def handle_get_global_boundary_center(self, req):
        """Service handler to return the global boundary center."""
        response = GlobalBoundaryCenterResponse()
        if self.global_boundary_center is not None:
            response.center = self.global_boundary_center
            response.radius = self.distance_threshold
            response.valid = True
        else:
            response.center = Point()
            response.valid = False
        return response


    def handle_get_local_boundary_center(self, req):
        """Service handler to return the local boundary center."""
        response = LocalBoundaryCenterResponse()
        if self.local_boundary_center is not None:
            response.center = self.local_boundary_center
            response.radius = self.local_boundary_radius
            response.valid = True
        else:
            response.center = Point()
            response.valid = False
        return response


    def handle_get_litter_set(self, req):
        """Service handler to return the current set of litter."""
        response = GetLitterSetResponse()

        with self.litter_mutex:     
            if not self.set_litter:
                rospy.logwarn("No litter points in set_litter.")

            for litter_tuple in self.set_litter:
                litter_id, x, y, z = litter_tuple
                litter_point = LitterPoint()
                litter_point.id = litter_id
                litter_point.point.x = x
                litter_point.point.y = y
                litter_point.point.z = z

                rospy.loginfo(f"Adding litter to response: ID={litter_id}, x={x}, y={y}, z={z}")
                response.litter_points.append(litter_point)
        
        # Debug: Log the total count of litter points in the response
        rospy.loginfo(f"Total litter points in response: {len(response.litter_points)}")

        return response


    def handle_get_next_litter(self, req):
        """Service handler to pop the next litter target from the min-heap and return it as a LitterPoint."""
        response = GetNextLitterResponse()
        response.next_litter = LitterPoint()  # Initialize with an empty LitterPoint
        response.success = False  # Default to False in case there are no litter points
        
        with self.litter_mutex:
            if len(self.min_heap_litter) == 0:
                rospy.loginfo("No litter points in min_heap_litter.")
                response.next_litter = LitterPoint()
                response.success = False
                return response

            # Pop the closest litter point from the heap
            try:
                _, _, self.next_litter = self.pop_min_heap()

                # Process next litter
                self.process_target_litter(self.next_litter)

                # Fill in the response LitterPoint
                litter_msg = LitterPoint()
                litter_msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
                litter_msg.point = self.next_litter.point
                litter_msg.id = self.next_litter.id

                response.next_litter = litter_msg
                response.success = True

                rospy.loginfo(f"Next litter to navigate: ID={litter_msg.id}, Position={litter_msg.point}")
            
            except IndexError:
                # Handle case where pop_min_heap fails
                rospy.logwarn("Attempted to pop from an empty min_heap_litter.")
                response.success = False

        return response


    def handle_get_next_target_litter(self, req):
        """Service handler to get the next target litter and return it as a LitterPoint."""
        if self.next_litter:  # Check if there is a next target waypoint
            # Return the next waypoint without removing it from the heap
            response = GetNextTargetLitterResponse()
            litter_msg = LitterPoint()
            litter_msg.header = Header(stamp=rospy.Time.now(), frame_id="map")
            litter_msg.point = self.next_litter.point
            litter_msg.id = self.next_litter.id
            response.litter = litter_msg
            response.success = True
            return response


    def handle_remove_litter(self, req):    
        """Service handler to remove litter from the litter manager memory (Litter is "Cleared")"""
        response = RemoveLitterResponse()
        response.success = self.delete_litter(req.litter)
        return response 


    def handle_clear_previous_job(self, req):
        """Service callback to stop and clear the previous litter picking job."""
        self.global_boundary_center = None  # Reset the global boundary center
        self.local_boundary_center = None   # Reset the local boundary center
        self.set_litter = set()             # Reset the set litter
        
        # Clear litter memory by calling the service
        try:
            response = self.clear_litter_memory_srv()
            if not response.success:
                rospy.logwarn("Failed to clear litter memory.")
                return TriggerResponse(
                    success=False,
                    message="Failed to clear litter memory."
                )
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to clear litter memory failed: {e}")
            return TriggerResponse(
                success=False,
                message="Service call to clear litter memory failed."
            )
        
        # Republish the boundaries
        self.republish_global_boundary()
        self.republish_local_boundary()

        return TriggerResponse(
            success=True,
            message="Previous litter picking job cleared successfully"
        )



    def request_mode_switch(self, mode):
        """Threaded function to handle mode switch requests."""
        try:
            request = ModeSwitchRequest(mode=mode)
            response = self.req_mode(request)
            if response.success:
                rospy.loginfo(f"Successfully switched to mode {mode}.")
            else:
                rospy.logwarn(f"Failed to switch to mode {mode}: {response.message}")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call failed for mode switch to mode {mode}: {e}")



    def litter_point_to_tuple(self, litter_point):
        """Helper function to convert litter to tuple for storing in set (hashable)"""
        return (litter_point.id, litter_point.point.x, litter_point.point.y, litter_point.point.z)


    def detection_callback(self, detected_detailed_litter):
        """Callback function for when litter is detected (took into account the distance from existing litters)."""
        # Extract the LitterPoint data from the DetectedLitterPoint data
        detected_litter = detected_detailed_litter.litter_point

        # Convert detected litter point to a tuple (id, x, y, z)
        detected_litter_tuple = self.litter_point_to_tuple(detected_litter)

        # Check if global boundary needs to be established
        if self.global_boundary_center is None:
            # Set global boundary if the litter is within threshold distance
            if self.is_within_threshold(detected_litter.point, self.get_robot_position()):
                rospy.loginfo(f"Detected Litter is within threshold of {self.distance_threshold}")

                # Wait for robot to be in a mode not in LITTER_PICKING or LITTER_TRACKING mode, to prevent race conditions
                if self.get_robot_mode().mode == 3 or self.get_robot_mode().mode == 5: # TODO: Change the number to an enum label
                    rospy.loginfo("Did not register new litter because robot is in LITTER PICKING or LITTER_TRACKING mode")
                    return
                if self.get_robot_mode().mode == 6:
                    rospy.loginfo("Detected litter but robot in IDLE_LATCH mode")
                    return
                # Proceed to initialise LITTER_PICKING mode
                self.global_boundary_center = self.get_robot_position()
                self.start_pos = self.global_boundary_center

                # Populate `self.set_litter` with known litter within the global boundary
                known_litter = self.get_known_litter()
                rospy.loginfo(f"Global Boundary: {self.global_boundary_center}")
                rospy.loginfo(f"Known Litter: {known_litter}")

                with self.litter_mutex:
                    self.set_litter.clear()  # Clear any previous litter data
                    for litter in known_litter:
                        litter_tuple = self.litter_point_to_tuple(litter)
                        if self.is_within_threshold(litter.point, self.global_boundary_center):
                            self.set_litter.add(litter_tuple)

                    self.update_min_heap()

                # Request mode switch to LITTER_PICKING
                try:
                    # TODO: Fix the enum mode from number to the Title of the enum 
                    request = ModeSwitchRequest(mode=3) # '3' is the enum num for LITTER_PICKING mode
                    response = self.req_mode(request)

                    if not response.success:
                        rospy.logwarn("Unable to change robot controller to LITTER_PICKING mode")
                        return  # Ignore the rest of the code
                except rospy.ServiceException as e:
                    rospy.logwarn(f"Service call failed for mode switch to LITTER_PICKING: {e}")
                    return  # Exit on service call failure

                rospy.loginfo(f"Set litter initialized within global boundary: {self.set_litter}")
            else:
                rospy.loginfo("Detected litter is outside initial threshold; no boundary set.")
                return  # Ignore detection outside threshold
        else:
            # Check if the litter is within the global boundary
            if self.within_global_boundary(detected_litter.point):
                with self.litter_mutex:
                    # Add litter to `self.set_litter` if it’s not already there
                    if detected_litter_tuple not in self.set_litter:
                        self.set_litter.add(detected_litter_tuple)
                        self.update_min_heap()
                        rospy.loginfo(f"Added litter within global boundary: {detected_litter_tuple}")

            # Additional check: if within the local boundary, add to `self.set_litter`
            if self.local_boundary_center and self.local_boundary_radius > 0:
                distance_to_local_center = self.calculate_euclidean_distance(detected_litter.point, self.local_boundary_center)
                if distance_to_local_center <= self.local_boundary_radius:
                    with self.litter_mutex:
                        if detected_litter_tuple not in self.set_litter:
                            self.set_litter.add(detected_litter_tuple)
                            self.update_min_heap()
                            rospy.loginfo(f"Added litter within local boundary: {detected_litter_tuple}")

        # Clear the boundary if all litter within it is cleared
        self.check_and_clear_boundary()


    def get_known_litter(self):
        """Retrieve remembered litter positions from the GetLitterList service."""
        try:
            response = self.get_litter_list_srv()
            rospy.loginfo(f"Retrieved {len(response.remembered_litter.litter_points)} known litter positions.")
            return response.remembered_litter.litter_points
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call /litter_memory/get_litter_list service: {e}")
            return []

    
    def within_global_boundary(self, litter):
        """Checks if litter is within the global boundary"""
        # Set global boundary on the first detection
        if self.global_boundary_center is None:
            self.global_boundary_center = self.start_pos               # Initial center
    
            rospy.loginfo(f"Setting global boundary center at: {self.global_boundary_center}")

        # Calculate the Euclidean distance from litter to global boundary center
        distance = self.calculate_euclidean_distance(litter, self.global_boundary_center)
        
        # Check if litter is within the threshold
        return distance <= self.distance_threshold


    def is_within_threshold(self, litter, center):
        """Checks if litter is within threshold distance from the center"""
        return self.calculate_euclidean_distance(litter, center) <= self.distance_threshold


    def check_and_clear_boundary(self):
        """Clear the global boundary if all litter within it has been removed."""
        with self.litter_mutex:
            if len(self.set_litter) == 0:
                rospy.loginfo("All litter within the global boundary cleared.")
                self.global_boundary_center = None  # Reset the global boundary
                self.local_boundary_center = None   # Reset the local boundary
                
                # Trigger boundary marker updates
                self.trigger_global_boundary_marker()
                self.trigger_local_boundary_marker()
                
                # Since there is no more litter then remove it from next waypoint
                self.next_litter = None


    def compute_local_boundary_radius(self, litter_position):
        """Compute the radius of the local boundary based on distance from the global center."""
        # Calculate distance from the global boundary center
        distance_from_global = self.calculate_euclidean_distance(litter_position, self.start_pos)
        
        # Scale the radius based on distance, keeping it within [min_local_radius, max_local_radius]
        local_radius = max(self.max_local_radius - distance_from_global, self.min_local_radius)
        return local_radius


    def process_target_litter(self, target_litter):
        """Process the target litter and apply local boundary logic."""
        # Set the center of the local boundary to the target litter's position
        self.local_boundary_center = target_litter.point
        
        # Compute the local boundary radius
        self.local_boundary_radius = self.compute_local_boundary_radius(target_litter.point)
        
        # Call to publish the local boundary marker for visualization
        self.trigger_local_boundary_marker()
        
        # Check for additional litter within this local boundary
        for litter in self.get_known_litter():
            litter_position = litter.point
            distance_to_local_center = self.calculate_euclidean_distance(litter_position, self.local_boundary_center)
            
            if distance_to_local_center <= self.local_boundary_radius:
                # If the litter is within the local boundary, add it to the set and min_heap
                litter_tuple = self.litter_point_to_tuple(litter)
                if litter_tuple not in self.set_litter:
                    self.set_litter.add(litter_tuple)
                    self.push_min_heap(litter_tuple)  # Add to min_heap based on distance


    def calculate_euclidean_distance(self, point1, point2):
        """Calculates the euclidean distance between two points"""
        # Check if either point is None
        if point1 is None or point2 is None:
            rospy.logwarn("One of the points is None, cannot calculate distance.")
            return float('inf')  # Return a large distance if calculation is impossible
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)


    def get_robot_position(self):
        """Call the /get_amcl_pose service to get the robot's current position."""
        try:
            response = self.get_pose_srv()
            # Accessing position correctly within PoseWithCovarianceStamped
            return response.pose.pose.pose.position
        except rospy.ServiceException as e:
            rospy.logwarn("Service call to /get_amcl_pose failed, returning None for position.")
            return None


    def get_distance_to_litter(self, start_pos, litter):
        """Gets distance to the litter using ROS planner"""
        # Set up start and goal poses for the path planning request
        start = PoseStamped()
        start.header.frame_id = "map"
        start.pose.position = start_pos
        start.pose.orientation.w = 1.0  # Neutral orientation

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position = litter
        goal.pose.orientation.w = 1.0

        # Make a plan request to nav planner
        try:
            rospy.loginfo("Getting distance to litter using ROS Planner")
            plan_request = GetPlanRequest()
            plan_request.start = start
            plan_request.goal = goal
            plan_request.tolerance = 0.5

            # Call the navfn service
            plan_response = self.make_plan_srv(plan_request)

            # Extract plan
            plan_poses = plan_response.plan.poses

            if not plan_poses:
                rospy.logwarn("No path returned by move_base; returning infinite distance.")
                return float('inf')
            
            # Calculate distance
            distance = self.calculate_path_distance(plan_poses)

            rospy.loginfo(f"Distance from {start.pose.position} to {goal.pose.position} = {distance}")
            return distance 
        
        except rospy.ROSException as e:
            rospy.logwarn(f"Service /move_base/make_plan is unavailable or timed out: {e}")
            return float('inf')


    def calculate_path_distance(self, plan_poses):
        """Calculates the path distance of a plan"""
        # Sum distances between consecutive waypoints in the plan
        total_distance = 0.0
        for i in range(1, len(plan_poses)):
            prev = plan_poses[i - 1].pose.position
            curr = plan_poses[i].pose.position
            segment_distance = self.calculate_euclidean_distance(prev, curr)
            total_distance += segment_distance
        return total_distance


    def pop_min_heap(self):
        """Pop a tuple (distance, LitterPoint) from min_heap to get next litter target"""
        return heapq.heappop(self.min_heap_litter)


    def push_min_heap(self, litter_tuple):
        """Push a tuple (distance, LitterPoint) onto the min-heap."""
        litter_id, x, y, z = litter_tuple
        litter_point = LitterPoint()
        litter_point.id = litter_id
        litter_point.point = Point(x, y, z)

        # Calculate distance from start position
        distance = self.get_distance_to_litter(self.start_pos, litter_point.point)
        heapq.heappush(self.min_heap_litter, (distance, litter_id, litter_point))
        rospy.loginfo(f"Min Heap: {self.min_heap_litter}")


    def update_min_heap(self):
        """Rebuild the heap using current litter distances."""
        self.min_heap_litter.clear()
        for litter_tuple in self.set_litter:
            self.push_min_heap(litter_tuple)


    def delete_litter(self, litter):
        """Remove a litter from the memory (using ID) and set (using tuple)."""
        try:
            # Create a request for deleting litter by ID
            response = self.delete_litter_srv(DeleteLitterRequest(id=litter.id))
            if response.success:
                rospy.loginfo(f"Litter with ID {litter.id} deleted successfully.")
                
                # Remove from `set_litter`
                tuple_litter = self.litter_point_to_tuple(litter)
                self.set_litter.discard(tuple_litter)
                
                # Update min heap after removing a litter
                self.update_min_heap()

                # Check if we nede to clear boundary.
                self.check_and_clear_boundary()

            else:
                rospy.logwarn(f"Failed to delete litter with ID {litter.id}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to delete litter ID {litter.id} failed: {e}")
            return False


    def trigger_global_boundary_marker(self):
        """Trigger the global boundary marker publishing."""
        try:
            response = self.republish_global_boundary()
            if response.success:
                rospy.loginfo("Global boundary marker published.")
            else:
                rospy.logwarn(f"Failed to publish global boundary marker: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /republish_global_boundary failed: {e}")


    # Replace `publish_local_boundary_marker` with a call to the `/republish_local_boundary` service
    def trigger_local_boundary_marker(self):
        """Trigger the local boundary marker publishing."""
        try:
            response = self.republish_local_boundary()
            if response.success:
                rospy.loginfo("Local boundary marker published.")
            else:
                rospy.logwarn(f"Failed to publish local boundary marker: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /republish_local_boundary failed: {e}")


    def handle_has_litter_to_clear(self, req):
        """Service handler to check if there is litter left to clear."""
        response = HasLitterToClearResponse()
        try: 
            response.has_litter = self.has_litter_to_clear()
            rospy.loginfo(f"Has litter to clear: {response.has_litter}")
            response.success = True
        except:
            response.success = False
        return response


    def has_litter_to_clear(self):
        """Checks if there are still litter to clear"""
        return len(self.set_litter) > 0