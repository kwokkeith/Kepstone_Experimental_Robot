#include "bumperbot_controller/litter_tracker.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "litter_tracker_node");

    // Create a NodeHandle
    ros::NodeHandle nh;

    // Instantiate the LitterTracker class
    LitterTracker tracker(nh);

    // Run the spin loop
    tracker.spin();

    return 0;
}
