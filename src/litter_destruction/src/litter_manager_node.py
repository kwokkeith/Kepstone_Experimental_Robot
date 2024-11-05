import rospy
from litter_destruction.litter_manager import LitterManager 

def main():
    # Initialize the ROS node
    rospy.init_node('litter_manager_node')

    # Instantiate LitterManager
    distance_threshold = rospy.get_param("~distance_threshold", 5.0)
    min_local_radius = rospy.get_param("~min_local_radius", 1.0)
    max_local_radius = rospy.get_param("~max_local_radius", 3.0)
    
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
