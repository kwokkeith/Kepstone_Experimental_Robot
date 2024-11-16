#include "ros/ros.h"
#include "bumperbot_controller/move_manager.h"

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "move_manager_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Instantiate the MoveManager object
    MoveManager move_manager(nh);

    ROS_INFO("Move Manager Node is up and running.");

    // Spin the MoveManager
    move_manager.spin();

    return 0;
}
