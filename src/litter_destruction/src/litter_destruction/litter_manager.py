import rospy
import threading
import heapq
import math
from geometry_msgs.msg import Point, PoseStamped
from bumperbot_detection.msg import  LitterPoint
from nav_msgs.srv import GetPlan
from navigation.srv import GetAmclPose
from bumperbot_detection.srv import DeleteLitter, GetLitterList
from visualization_msgs.msg import Marker


class LitterManager:
    def __init__(self, distance_threshold, robot, min_local_radius=1, max_local_radius=5):
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

        rospy.Subscriber("/litter_memory/new_litter", LitterPoint, self.detection_callback)
        self.make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.get_litter_list_srv = rospy.ServiceProxy('/litter_memory/get_litter_list', GetLitterList)         # Define the service client for getting litter list
        self.delete_litter_srv = rospy.ServiceProxy('/litter_memory/delete_litter', DeleteLitter)              # Define the service client for deleting litter
        self.global_boundary_marker_pub = rospy.Publisher("global_boundary_marker", Marker, queue_size=10)     # Define publisher for global boundary marker
        self.local_boundary_marker_pub = rospy.Publisher("local_boundary_marker", Marker, queue_size=10)       # Define publisher for local boundary marker

        # Initialize service client for getting the current AMCL position
        rospy.wait_for_service('/get_amcl_pose')
        self.get_pose_srv = rospy.ServiceProxy('/get_amcl_pose', GetAmclPose)



    def litter_point_to_tuple(self, litter_point):
        """Helper function to convert litter to tuple for storing in set"""
        return (litter_point.id, litter_point.point.x, litter_point.point.y, litter_point.point.z)


    def detection_callback(self, detected_litter):
        """Callback function for when litter is detected"""
        # Extract Point data from PointStamped
        # (id, x, y, z)
        detected_litter_tuple = self.litter_point_to_tuple(detected_litter)

        # Establish a global boundary only if litter is within the threshold distance
        if self.global_boundary_center is None:
            # First detection: Set global boundary center if within threshold
            if self.is_within_threshold(detected_litter.point, self.get_robot_position()):
                rospy.loginfo(f"Detected Litter is within threshold of {self.distance_threshold}")
                # Start running the global boundary clearing of litter
                # Entry code of cleaning algorithm by setting global boundary
                self.global_boundary_center = self.get_robot_position()
                self.start_pos = self.global_boundary_center
                self.publish_global_boundary_marker()  # Publish the marker after setting the boundary center

                # Get known litter from robot memory
                known_litter = self.get_known_litter()

                rospy.loginfo(f"Global Boundary: {self.global_boundary_center}")
                rospy.loginfo(f"Known Litter: {known_litter}")

                with self.litter_mutex:
                    rospy.loginfo(f"Collected litter mutex")
                    self.set_litter.clear() # Clean the set litter if this is a new run
                    # Add only known litter within the global boundary to the local set
                    for litter in known_litter:
                        litter_tuple = self.litter_point_to_tuple(litter)
                        if self.is_within_threshold(litter.point, self.global_boundary_center):
                            self.set_litter.add(litter_tuple)
                    
                    rospy.loginfo(f"updated set litter: {self.set_litter}")

                    self.update_min_heap() # Rebuild heap with new litter start position

                    rospy.loginfo(f"Updated min heap: {self.min_heap_litter}")
                    rospy.loginfo(f"Global boundary set at {self.global_boundary_center}")

            else:
                rospy.loginfo("Detected litter is outside initial threshold; no boundary set.")
                return  # Ignore this detection
            
        # if litter is within global boundary
        elif self.within_global_boundary(detected_litter.point):
            with self.litter_mutex:
                if detected_litter.id not in {id for (id, _, _, _) in self.set_litter}:
                    self.set_litter.add(detected_litter_tuple)

                    # Push the new detected litter with its distance to the min_heap 
                    self.update_min_heap()  

        # Clear the boundary if all litter within it is cleared
        self.check_and_clear_boundary()


    def get_known_litter(self):
        """Retrieve remembered litter positions from the GetLitterList service."""
        rospy.wait_for_service('/litter_memory/get_litter_list')
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
                self.global_boundary_center = None  # Reset the boundary
                self.publish_global_boundary_marker()


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
                    self.push_min_heap(litter)  # Add to min_heap based on distance


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

        # Make a plan request to move_base/make_plan
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=5)  # Wait with a 5-second timeout
            plan = self.make_plan_srv(start=start, goal=goal, tolerance=0.5).plan.poses
            distance = self.calculate_path_distance(plan)
            # rospy.loginfo(f"Distance from \n{start.pose.position} to \n{goal.pose.position}\n = {distance}")
            rospy.loginfo(f"\n\nset_litter: {self.set_litter}")
            return distance 
        except rospy.ROSException as e:
            rospy.logwarn("Service /move_base/make_plan is unavailable or timed out.")
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


    def push_min_heap(self, litter_tuple):
        """Push a tuple (distance, LitterPoint) onto the min-heap."""
        litter_id, x, y, z = litter_tuple
        litter_point = LitterPoint()
        litter_point.id = litter_id
        litter_point.point = Point(x, y, z)

        # Calculate distance from start position
        distance = self.get_distance_to_litter(self.start_pos, litter_point.point)
        heapq.heappush(self.min_heap_litter, (distance, litter_point))
        rospy.loginfo(f"Min Heap: {self.min_heap_litter}")


    def update_min_heap(self):
        """Rebuild the heap using current litter distances."""
        self.min_heap_litter.clear()
        for litter_tuple in self.set_litter:
            self.push_min_heap(litter_tuple)


    def delete_litter(self, litter_id):
        """Remove a litter by ID from the memory and set."""
        rospy.wait_for_service('/litter_memory/delete_litter')
        try:
            # Create a request for deleting litter by ID
            response = self.delete_litter_srv(DeleteLitter(id=litter_id))
            if response.success:
                rospy.loginfo(f"Litter with ID {litter_id} deleted successfully.")
                # Remove from `set_litter`
                self.set_litter = {l for l in self.set_litter if l[0] != litter_id}
            else:
                rospy.logwarn(f"Failed to delete litter with ID {litter_id}: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to delete litter ID {litter_id} failed: {e}")
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

        if self.local_boundary_center:
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
        else:
            # No local boundary; clear the marker
            marker.action = Marker.DELETE

        # Publish the marker
        self.local_boundary_marker_pub.publish(marker)

    
    def has_litter_to_clear(self):
        """Checks if there are still litter to clear"""
        return len(self.set_litter) > 0