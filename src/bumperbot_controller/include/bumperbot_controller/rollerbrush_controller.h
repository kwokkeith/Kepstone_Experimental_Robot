#ifndef ROLLERBRUSH_CONTROLLER_H
#define ROLLERBRUSH_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <bumperbot_controller/SetRollerBrushPower.h>
#include <bumperbot_controller/GetRollerBrushPower.h>
#include <bumperbot_controller/SetRollerBrushDirection.h>
#include <bumperbot_controller/GetRollerBrushDirection.h>

class RollerBrushController
{
public:
    RollerBrushController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher rollerbrush_power_pub_;
    ros::Publisher rollerbrush_direction_pub_;
    ros::ServiceServer set_rollerbrush_power_service_;
    ros::ServiceServer get_rollerbrush_power_service_;
    ros::ServiceServer set_rollerbrush_direction_service_;
    ros::ServiceServer get_rollerbrush_direction_service_;

    float current_power_;
    std::string current_direction_;

    // Callback function for the service to set Rollerbrush Power
    bool setRollerBrushPower(bumperbot_controller::SetRollerBrushPower::Request& req, 
                        bumperbot_controller::SetRollerBrushPower::Response& res);

    bool getRollerBrushPower(bumperbot_controller::GetRollerBrushPower::Request& req, 
                        bumperbot_controller::GetRollerBrushPower::Response& res);

    bool setRollerBrushDirection(bumperbot_controller::SetRollerBrushDirection::Request& req, 
                        bumperbot_controller::SetRollerBrushDirection::Response& res);

    bool getRollerBrushDirection(bumperbot_controller::GetRollerBrushDirection::Request& req,   
                        bumperbot_controller::GetRollerBrushDirection::Response& res);
};

#endif 
