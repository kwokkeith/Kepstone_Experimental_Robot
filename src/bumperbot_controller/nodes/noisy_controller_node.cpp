#include <ros/ros.h>
#include "bumperbot_controller/noisy_controller.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "noisy_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 
    
    double wheel_radius, wheel_separation, wheel_radius_error, wheel_separation_error;
    double wheel_radius_w_error, wheel_separation_w_error;
    pnh.getParam("/wheel/radius", wheel_radius);
    pnh.getParam("/wheel/separation", wheel_separation);
    pnh.getParam("/wheel/radius_error", wheel_radius_error);
    pnh.getParam("/wheel/separation_error", wheel_separation_error);

    wheel_radius_w_error = wheel_radius + wheel_radius_error;
    wheel_separation_w_error = wheel_separation + wheel_separation_error;
    
    NoisyController controller(nh, wheel_radius_w_error, wheel_separation_w_error);

    ros::spin();

    return 0;
}   