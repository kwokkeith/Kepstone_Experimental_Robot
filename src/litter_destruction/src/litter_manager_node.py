import rospy
from litter_destruction.litter_manager import LitterManager 

def main():
    # Initialize the ROS node
    rospy.init_node('litter_manager_node')

    # Instantiate LitterManager 
    distance_threshold = rospy.get_param("/litter_manager/distance_threshold")
    min_local_radius = rospy.get_param("/litter_manager/min_local_radius")
    max_local_radius = rospy.get_param("/litter_manager/max_local_radius")
    
    # Pass parameters to the LitterManager instance
    manager = LitterManager(distance_threshold=distance_threshold, 
                            min_local_radius=min_local_radius, 
                            max_local_radius=max_local_radius)
    
    rospy.loginfo("LitterManager node started.")
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
