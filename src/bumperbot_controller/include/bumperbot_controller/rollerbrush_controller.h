#ifndef ROLLERBRUSH_CONTROLLER_H
#define ROLLERBRUSH_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <bumperbot_controller/SetRollerBrushPower.h>
#include <bumperbot_controller/GetRollerBrushPower.h>
#include <bumperbot_controller/SetRollerBrushPosition.h>
#include <bumperbot_controller/GetRollerBrushPosition.h>
#include <bumperbot_controller/RollerBrushPosFeedback.h>
#include <bumperbot_controller/RollerBrushPowerFeedback.h>

class RollerBrushController
{
public:
    RollerBrushController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher rollerbrush_power_pub_;
    ros::Publisher rollerbrush_position_pub_;
    ros::ServiceServer set_rollerbrush_power_service_;
    ros::ServiceServer get_rollerbrush_power_service_;
    ros::ServiceServer set_rollerbrush_position_service_;
    ros::ServiceServer get_rollerbrush_position_service_;

    float current_power_;
    std::string current_position_;

    // Callback function for the service to set Rollerbrush Power
    bool setRollerBrushPower(bumperbot_controller::SetRollerBrushPower::Request& req, 
                        bumperbot_controller::SetRollerBrushPower::Response& res);

    bool getRollerBrushPower(bumperbot_controller::GetRollerBrushPower::Request& req, 
                        bumperbot_controller::GetRollerBrushPower::Response& res);

    bool setRollerBrushPosition(bumperbot_controller::SetRollerBrushPosition::Request& req, 
                        bumperbot_controller::SetRollerBrushPosition::Response& res);

    bool getRollerBrushPosition(bumperbot_controller::GetRollerBrushPosition::Request& req,   
                        bumperbot_controller::GetRollerBrushPosition::Response& res);
};

#endif 
