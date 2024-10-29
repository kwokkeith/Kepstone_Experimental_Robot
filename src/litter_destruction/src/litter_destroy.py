import rospy
import heapq
import time
from litter_destruction.litter_manager import LitterManager
from geometry_msgs.msg import Point
from navigation.srv import GetAmclPose
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

def main():
    # Initialize the ROS node
    rospy.init_node('litter_management_node', log_level=rospy.INFO)
    
    rospy.loginfo("Node started: Initializing robot and manager...")
    
    robot = Robot()
    
     # Set the minimum and maximum local boundary radius
    min_local_radius = 0.5  # Minimum size of the local boundary
    max_local_radius = 1.5  # Maximum size based on distance

    manager = LitterManager(distance_threshold=2.0, 
                            robot=robot, 
                            min_local_radius=min_local_radius, 
                            max_local_radius=max_local_radius)
    
    rospy.loginfo("Robot and manager initialized. Starting main loop.")

    while not rospy.is_shutdown():
        if manager.min_heap_litter and manager.set_litter:
            # Pop the closest litter item based on distance
            _, target_litter = heapq.heappop(manager.min_heap_litter)
            rospy.loginfo(f"Moving to litter at position: {target_litter.point}")

             # Set the local boundary and process additional litter within it
            manager.process_target_litter(target_litter)

            # Move robot to the target litter
            robot.move_to(target_litter.point)
            robot.navigate_client.wait_for_result()  # Wait until reached

            # Start executing cleaning procedure
            robot.destroy_litter()

            if manager.delete_litter(target_litter.id):
                rospy.loginfo(f"Litter with ID {target_litter.id} successfully deleted.")
                manager.set_litter.discard(target_litter)
                manager.start_pos = target_litter.point
                manager.update_min_heap()

if __name__ == "__main__":
    main()