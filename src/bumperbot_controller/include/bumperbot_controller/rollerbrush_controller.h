#ifndef ROLLERBRUSH_CONTROLLER_H
#define ROLLERBRUSH_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <bumperbot_controller/SetRollerBrushPower.h>
#include <bumperbot_controller/GetRollerBrushPower.h>

class RollerBrushController
{
public:
    RollerBrushController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher rollerbrush_pub_;
    ros::ServiceServer set_rollerbrush_service_;
    ros::ServiceServer get_rollerbrush_service_;

    float current_power_;

    // Callback function for the service to set Rollerbrush Power
    bool setRollerBrushPower(bumperbot_controller::SetRollerBrushPower::Request& req, 
                        bumperbot_controller::SetRollerBrushPower::Response& res);

    bool getRollerBrushPower(bumperbot_controller::GetRollerBrushPower::Request& req, 
                        bumperbot_controller::GetRollerBrushPower::Response& res);
};

#endif 
