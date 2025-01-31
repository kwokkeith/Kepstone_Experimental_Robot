#include <ros/ros.h>
#include "bumperbot_controller/simple_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_controller");
    ros::NodeHandle nh;
    // ("~") Creates a private nodehandle namespace for this simple_controller node, parameters and topics are private to this node
    ros::NodeHandle pnh("~"); 
    
    double wheel_radius, wheel_separation;
    pnh.getParam("/wheel/radius", wheel_radius);
    pnh.getParam("/wheel/separation", wheel_separation);
    
    SimpleController controller(nh, wheel_radius, wheel_separation);

    ros::spin();

    return 0;
}