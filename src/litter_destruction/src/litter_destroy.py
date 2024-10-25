import rospy
import threading
import heapq
import math
import time
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from bumperbot_detection.msg import LitterList, LitterPoint
from nav_msgs.srv import GetPlan
from navigation.srv import GetAmclPose
from bumperbot_detection.srv import AddLitter, DeleteLitter, GetLitterList
from litter_destruction.msg import NavigateAction, NavigateGoal
from actionlib import SimpleActionClient

class Robot:
    def __init__(self):
        self.navigate_client = SimpleActionClient('navigate_to_litter', NavigateAction)
        self.destroy_client = SimpleActionClient('destroy_litter', NavigateAction)
        self.current_position = Point()  # Cached robot position
        
        # Initialize service client for getting the current AMCL position
        rospy.wait_for_service('/get_amcl_pose')
        self.get_pose_srv = rospy.ServiceProxy('/get_amcl_pose', GetAmclPose)

    def move_to(self, target):
        goal = NavigateGoal(target=target)
        self.navigate_client.send_goal(goal)

    def get_current_position(self):
        """Call the /get_amcl_pose service to get the robot's current position."""
        try:
            response = self.get_pose_srv()
            # Accessing position correctly within PoseWithCovarianceStamped
            return response.pose.pose.pose.position
        except rospy.ServiceException as e:
            rospy.logwarn("Service call to /get_amcl_pose failed, returning None for position.")
            return None

    def destroy_litter(self):
        # Initiate destroy action
        self.destroy_client.send_goal(NavigateGoal(target=self.get_current_position))
        time.sleep(5000) # Simulate cleaning (5 seconds) 

class LitterManager:
    def __init__(self, distance_threshold, robot):
        self.set_litter = set()
        self.min_heap_litter = []
        self.start_pos = Point()
        self.litter_mutex = threading.Lock()
        self.global_boundary_center = None            # Center of global boundary
        self.distance_threshold = distance_threshold  # Radius for the global boundary

        rospy.Subscriber("base_frame/detected_object_coordinates", PointStamped, self.detection_callback)
        self.make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.get_litter_list_srv = rospy.ServiceProxy('/litter_memory/get_litter_list', GetLitterList)  # Define the service client for getting litter list
        self.delete_litter_srv = rospy.ServiceProxy('/litter_memory/delete_litter', DeleteLitter)       # Define the service client for deleting litter
        self.robot = robot

    def litter_point_to_tuple(self, litter):
        return (litter.x, litter.y, litter.z)


    def detection_callback(self, detected_litter_stamped):
        # Extract Point data from PointStamped
        detected_litter = detected_litter_stamped.point
        detected_litter_tuple = self.litter_point_to_tuple(detected_litter)

        # Establish a global boundary only if litter is within the threshold distance
        if self.global_boundary_center is None:
            # First detection: Set global boundary center if within threshold
            if self.is_within_threshold(detected_litter, self.get_robot_position()):
                rospy.loginfo(f"Detected Litter is within threshold of {self.distance_threshold}")
                # Start running the global boundary clearing of litter
                # Entry code of cleaning algorithm by setting global boundary
                self.global_boundary_center = self.get_robot_position()
                self.start_pos = self.global_boundary_center

                # Get known litter from robot memory
                known_litter = self.get_known_litter()

                rospy.loginfo(f"Global Boundary: {self.global_boundary_center}")
                rospy.loginfo(f"Known Litter: {known_litter}")

                with self.litter_mutex:
                    rospy.loginfo(f"Collected litter mutex")
                    self.set_litter.clear() # Clean the set litter if this is a new run
                    # Add only known litter within the global boundary to the local set
                    for litter in known_litter:
                        rospy.loginfo(f"Checking {litter} \n within global boundary")
                        litter = litter.point
                        litter_tuple = self.litter_point_to_tuple(litter)
                        if self.is_within_threshold(litter, self.global_boundary_center):
                            self.set_litter.add(litter_tuple)
                    
                    rospy.loginfo(f"updated set litter: {self.set_litter}")

                    self.update_min_heap() # Rebuild heap with new litter start position

                    rospy.loginfo(f"Updated min heap: {self.min_heap_litter}")
                    rospy.loginfo(f"Global boundary set at {self.global_boundary_center}")

            else:
                rospy.loginfo("Detected litter is outside initial threshold; no boundary set.")
                return  # Ignore this detection
        # if litter is within global boundary
        elif self.within_global_boundary(detected_litter):
            with self.litter_mutex:
                if detected_litter_tuple not in self.set_litter:
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
        # Set global boundary on the first detection
        if self.global_boundary_center is None:
            self.global_boundary_center = self.get_robot_position()  # Initial center
            rospy.loginfo(f"Setting global boundary center at: {self.global_boundary_center}")

        # Calculate the Euclidean distance from litter to global boundary center
        distance = self.calculate_euclidean_distance(litter, self.global_boundary_center)
        
        # Check if litter is within the threshold
        return distance <= self.distance_threshold

    def is_within_threshold(self, litter, center):
        return self.calculate_euclidean_distance(litter, center) <= self.distance_threshold

    def check_and_clear_boundary(self):
        """Clear the global boundary if all litter within it has been removed."""
        with self.litter_mutex:
            if len(self.set_litter) == 0:
                rospy.loginfo("All litter within the global boundary cleared.")
                self.global_boundary_center = None  # Reset the boundary

    def calculate_euclidean_distance(self, point1, point2):
        # Check if either point is None
        if point1 is None or point2 is None:
            rospy.logwarn("One of the points is None, cannot calculate distance.")
            return float('inf')  # Return a large distance if calculation is impossible
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)

    def get_robot_position(self):
        return self.robot.get_current_position()

    def get_distance_to_litter(self, start_pos, litter):
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
            rospy.loginfo(f"Distance from \n{start.pose.position} to \n{goal.pose.position}\n = {distance}")

            return self.calculate_path_distance(plan)
        except rospy.ROSException as e:
            rospy.logwarn("Service /move_base/make_plan is unavailable or timed out.")
            return float('inf')

    def calculate_path_distance(self, plan):
        # Sum distances between consecutive waypoints in the plan
        total_distance = 0.0
        for i in range(1, len(plan)):
            prev = plan[i - 1].pose.position
            curr = plan[i].pose.position
            segment_distance = self.calculate_euclidean_distance(prev, curr)
            total_distance += segment_distance
        return total_distance

    def push_min_heap(self, litter_tuple):
        # Pushes the litter onto the min heap with the distance from the start position
        litter_point = Point(*litter_tuple)
        dist = self.get_distance_to_litter(self.start_pos, litter_point) # Get distance to litter from starting position
        heapq.heappush(self.min_heap_litter, (dist, litter_point))

    def update_min_heap(self):
        # Clear and rebuild min heap with updated start_pos distances
        self.min_heap_litter.clear()
        for litter in self.set_litter:
            self.push_min_heap(litter)

def main():
    # Initialize the ROS node
    rospy.init_node('litter_management_node', log_level=rospy.INFO)
    
    rospy.loginfo("Node started: Initializing robot and manager...")
    
    robot = Robot()
    manager = LitterManager(distance_threshold=2.0, robot=robot)
    
    rospy.loginfo("Robot and manager initialized. Starting main loop.")

    while not rospy.is_shutdown():
        # Ensure heap is populated
        if manager.min_heap_litter and manager.set_litter:
            # Select the closest litter based on min heap
            _, target_point = heapq.heappop(manager.min_heap_litter)
            rospy.loginfo(f"Moving to litter at position: {target_point}")

            # Move robot to the target litter
            robot.move_to(target_point)
            robot.navigate_client.wait_for_result()  # Wait until reached

            # Start executing cleaning procedure
            robot.destroy_litter()

            # Remove target from memory and update start position
            with manager.litter_mutex:
                rospy.wait_for_service('/litter_memory/delete_litter')
                try:
                    response = manager.delete_litter_srv(Point(*target_tuple))
                    if response.success:
                        rospy.loginfo(f"Litter at position {target_tuple} successfully deleted.")
                    else:
                        rospy.logwarn(f"Failed to delete litter at position {target_tuple}.")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

                manager.set_litter.discard(target_tuple)
                manager.start_pos = target_point  # Update start position to cleared litter position

            # Rebuild the heap with updated start position
            manager.update_min_heap()

            # Wait until cleaning procedure is done
            robot.destroy_client.wait_for_result()  

if __name__ == "__main__":
    main()

