import rospy
import heapq
import time
from litter_destruction.litter_manager import LitterManager
from geometry_msgs.msg import Point
from navigation.srv import GetAmclPose
from actionlib import SimpleActionClient


def main():
    # Initialize the ROS node
    rospy.init_node('litter_management_node', log_level=rospy.INFO)
    
    rospy.loginfo("Node started: Initializing robot and manager...")
    
    
     # Set the minimum and maximum local boundary radius
    min_local_radius = 0.5  # Minimum size of the local boundary
    max_local_radius = 1.5  # Maximum size based on distance

    manager = LitterManager(distance_threshold=2.0,  
                            min_local_radius=min_local_radius, 
                            max_local_radius=max_local_radius)
    
    rospy.loginfo("Robot and manager initialized. Starting main loop.")

    while not rospy.is_shutdown():
        if manager.min_heap_litter and manager.set_litter:
            # Pop the closest litter item based on distance
            _, target_litter = manager.pop_min_heap() 
            rospy.loginfo(f"Moving to litter at position: \n{target_litter.point}")

             # Set the local boundary and process additional litter within it
            manager.process_target_litter(target_litter)

            ### Move robot to the target litter
            # robot.move_to(target_litter.point)
            # robot.navigate_client.wait_for_result()  # Wait until reached

            ### Start executing cleaning procedure
            # robot.destroy_litter()

            time.sleep(10) # Simulate robot moving to litter

            if manager.delete_litter(target_litter):
                rospy.loginfo(f"Litter with ID {target_litter.id} successfully deleted.")
                manager.start_pos = target_litter.point
                manager.update_min_heap()

if __name__ == "__main__":
    main()