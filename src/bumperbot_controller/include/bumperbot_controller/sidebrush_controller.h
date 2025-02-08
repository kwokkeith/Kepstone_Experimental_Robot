#ifndef SIDEBRUSH_CONTROLLER_H
#define SIDEBRUSH_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <bumperbot_controller/SetSideBrushSpeed.h>
#include <bumperbot_controller/GetSideBrushSpeed.h>
#include <bumperbot_controller/SetSideBrushDirection.h>
#include <bumperbot_controller/GetSideBrushDirection.h>

class SideBrushController
{
public:
    SideBrushController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher sidebrush_speed_pub_;
    ros::Publisher sidebrush_direction_pub_;
    ros::ServiceServer set_sidebrush_speed_service_;
    ros::ServiceServer get_sidebrush_speed_service_;
    ros::ServiceServer set_sidebrush_direction_service_;
    ros::ServiceServer get_sidebrush_direction_service_;

    float current_speed_;
    std::string current_direction_;

    // Callback function for the service to set Sidebrush Speed
    bool setSideBrushSpeed(bumperbot_controller::SetSideBrushSpeed::Request& req, 
                        bumperbot_controller::SetSideBrushSpeed::Response& res);

    bool getSideBrushSpeed(bumperbot_controller::GetSideBrushSpeed::Request& req, 
                        bumperbot_controller::GetSideBrushSpeed::Response& res);

    bool setSideBrushDirection(bumperbot_controller::SetSideBrushDirection::Request& req, 
                        bumperbot_controller::SetSideBrushDirection::Response& res);

    bool getSideBrushDirection(bumperbot_controller::GetSideBrushDirection::Request& req,   
                        bumperbot_controller::GetSideBrushDirection::Response& res);
};

#endif 
