import rospy
import threading
import heapq
import math
from geometry_msgs.msg import Point, PoseStamped
from bumperbot_detection.msg import  LitterPoint
from std_msgs.msg import Header
from nav_msgs.srv import GetPlan
from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from navigation.srv import GetAmclPose
from bumperbot_detection.srv import DeleteLitterRequest, DeleteLitter, GetLitterList
from visualization_msgs.msg import Marker
from litter_destruction.srv import GlobalBoundaryCenter, GlobalBoundaryCenterResponse
from litter_destruction.srv import GetLitterSet, GetLitterSetResponse
from litter_destruction.srv import GetNextLitter, GetNextLitterResponse
from litter_destruction.srv import RemoveLitter, RemoveLitterRequest, RemoveLitterResponse
from litter_destruction.srv import HasLitterToClear, HasLitterToClearResponse
from litter_destruction.srv import GetNextTargetLitter, GetNextTargetLitterResponse
from bumperbot_controller.srv import ModeSwitch, ModeSwitchRequest

class LitterManager:
    def __init__(self, distance_threshold=5, min_local_radius=1, max_local_radius=3):
        self.set_litter = set()
        self.min_heap_litter = []
        self.start_pos = Point()
        self.litter_mutex = threading.Lock()
        self.global_boundary_center = None            # Center of global boundary
        self.distance_threshold = distance_threshold  # Radius for the global boundary
        self.local_boundary_center = None             # Center of local boundary
        self.min_local_radius = min_local_radius      # Min radius of local boundary
        self.max_local_radius = max_local_radius      # Max radius of local boundary
        self.local_boundary_radius = 0                # Radius of local boundary
        self.planner_service = "/planner_only/navfn_planner/make_plan"
        self.next_litter = None                       # To store the next litter to continuously publish

        ## Publisher
        self.global_boundary_marker_pub = rospy.Publisher("global_boundary_marker", Marker, queue_size=10)              # Define publisher for global boundary marker
        self.local_boundary_marker_pub  = rospy.Publisher("local_boundary_marker", Marker, queue_size=10)               # Define publisher for local boundary marker

        ## Subscribers
        rospy.Subscriber("/litter_memory/new_litter", LitterPoint, self.detection_callback)
        
        ## Service Clients
        rospy.wait_for_service(self.planner_service)
        rospy.wait_for_service('/litter_memory/get_litter_list')
        rospy.wait_for_service('/litter_memory/delete_litter')
        rospy.wait_for_service('/get_amcl_pose')
        rospy.wait_for_service('/robot_controller/mode_switch')

        self.make_plan_srv       = rospy.ServiceProxy(self.planner_service, MakeNavPlan)                       # Define service client to make plans using planner_only move_base node (not actual move_base)
        self.get_litter_list_srv = rospy.ServiceProxy('/litter_memory/get_litter_list', GetLitterList)         # Define the service client for getting litter list
        self.delete_litter_srv   = rospy.ServiceProxy('/litter_memory/delete_litter', DeleteLitter)            # Define the service client for deleting litter
        self.get_pose_srv        = rospy.ServiceProxy('/get_amcl_pose', GetAmclPose)                           # Define service client for getting amcl pose
        self.req_mode            = rospy.ServiceProxy('/robot_controller/mode_switch', ModeSwitch)             # Define service client for changing robot controller to litter mode

        ## Service Servers
        rospy.Service("/litter_manager/get_global_boundary_center", GlobalBoundaryCenter, self.handle_get_global_boundary_center)   # Define service server to get global boundary center
        rospy.Service("/litter_manager/get_litter_set", GetLitterSet, self.handle_get_litter_set)                                   # Define service server to get litter set
        rospy.Service("/litter_manager/get_next_litter", GetNextLitter, self.handle_get_next_litter)                                # Define service server to get next litter
        rospy.Service("/litter_manager/has_litter_to_clear", HasLitterToClear, self.handle_has_litter_to_clear)                     # Define service server to check if there is still litter in set
        rospy.Service("/litter_manager/delete_litter", RemoveLitter, self.handle_remove_litter)                                     # Define service server to delete litter from litter manager
        rospy.Service("/litter_manager/next_waypoint", GetNextTargetLitter, self.handle_get_next_target_litter)                     # Define service server to get the next target litter


    def handle_get_global_boundary_center(self, req):
        """Service handler to return the global boundary center."""
        response = GlobalBoundaryCenterResponse()
        if self.global_boundary_center is not None:
            response.center = self.global_boundary_center
            response.valid = True
        else:
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
            if not self.min_heap_litter:
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


    def litter_point_to_tuple(self, litter_point):
        """Helper function to convert litter to tuple for storing in set (hashable)"""
        return (litter_point.id, litter_point.point.x, litter_point.point.y, litter_point.point.z)


    def detection_callback(self, detected_litter):
        """Callback function for when litter is detected."""
        # Convert detected litter point to a tuple (id, x, y, z)
        detected_litter_tuple = self.litter_point_to_tuple(detected_litter)

        # Check if global boundary needs to be established
        if self.global_boundary_center is None:
            # Set global boundary if the litter is within threshold distance
            if self.is_within_threshold(detected_litter.point, self.get_robot_position()):
                rospy.loginfo(f"Detected Litter is within threshold of {self.distance_threshold}")

                self.global_boundary_center = self.get_robot_position()
                self.start_pos = self.global_boundary_center
                self.publish_global_boundary_marker()

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
                    request = ModeSwitchRequest(mode=3) # '3' is the enum num for RobotMode 
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
            self.global_boundary_center = self.get_robot_position()  # Initial center
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
                self.publish_global_boundary_marker()
                self.publish_local_boundary_marker()
                
                # Since there is no more litter then remove it from next waypoint
                self.next_litter = None


    def compute_local_boundary_radius(self, litter_position):
        """Compute the radius of the local boundary based on distance from the global center."""
        # Calculate distance from the global boundary center
        distance_from_global = self.calculate_euclidean_distance(litter_position, self.global_boundary_center)
        
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
        self.publish_local_boundary_marker()
        
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
            plan_request = MakeNavPlanRequest()
            plan_request.start = start
            plan_request.goal = goal
            # Call the navfn service
            plan_response = self.make_plan_srv(plan_request)

            # Check if a valid path was returned
            if not plan_response.path:
                rospy.logwarn("No path returned by the planner; returning infinite distance.")
                return float('inf')

            plan = plan_response.path
            distance = self.calculate_path_distance(plan)
            rospy.loginfo(f"Distance from \n{start.pose.position} to \n{goal.pose.position}\n = {distance}")
            #rospy.loginfo(f"\n\nset_litter: {self.set_litter}")
            return distance 
        except rospy.ROSException as e:
            rospy.logwarn(f"Service {self.planner_service} is unavailable or timed out.")
            return float('inf')


    def calculate_path_distance(self, plan):
        """Calculates the path distance of a plan"""
        # Sum distances between consecutive waypoints in the plan
        total_distance = 0.0
        for i in range(1, len(plan)):
            prev = plan[i - 1].pose.position
            curr = plan[i].pose.position
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
                
                # Check if we nede to clear boundary.
                self.check_and_clear_boundary()

            else:
                rospy.logwarn(f"Failed to delete litter with ID {litter.id}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to delete litter ID {litter.id} failed: {e}")
            return False


    def publish_global_boundary_marker(self):
        """To publish marker for global boundary visualization in RVIZ"""
        marker = Marker()
        marker.header.frame_id = "map"  # Use the same frame as the global boundary
        marker.header.stamp = rospy.Time.now()
        marker.ns = "global_boundary"
        marker.id = 0
        marker.type = Marker.CYLINDER  # Cylinder marker for circular representation

        if self.global_boundary_center is None:
            # Set action to DELETE to clear the marker in RViz if no boundary is set
            marker.action = Marker.DELETE
        else:
            # Otherwise, configure and add the marker to RViz
            marker.action = Marker.ADD

            # Set the position of the marker to the center of the global boundary
            marker.pose.position = self.global_boundary_center
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Define the scale of the marker (diameter in x and y, height in z)
            marker.scale.x = self.distance_threshold * 2  # Diameter for the x-axis
            marker.scale.y = self.distance_threshold * 2  # Diameter for the y-axis
            marker.scale.z = 0.1  # Small height for a flat circular marker

            # Set the color (RGBA) of the marker
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Semi-transparent

        # Publish the marker
        self.global_boundary_marker_pub.publish(marker)


    def publish_local_boundary_marker(self):
        """Publishes a marker to visualize the local boundary in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  # The frame for the marker, assuming 'map' frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "local_boundary"
        marker.id = 1
        marker.type = Marker.CYLINDER  # Use cylinder for circular boundary
        marker.action = Marker.ADD if self.local_boundary_center else Marker.DELETE

        if self.local_boundary_center is None:
            # No local boundary; clear the marker
            marker.action = Marker.DELETE
        else:
            # Set position at the center of the local boundary
            marker.pose.position = self.local_boundary_center
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale based on the local boundary radius
            marker.scale.x = self.local_boundary_radius * 2  # Diameter for x-axis
            marker.scale.y = self.local_boundary_radius * 2  # Diameter for y-axis
            marker.scale.z = 0.1  # Small height for a flat circle

            # Set color and transparency
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # Semi-transparent

        # Publish the marker
        self.local_boundary_marker_pub.publish(marker)


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