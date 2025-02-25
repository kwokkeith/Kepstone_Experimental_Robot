#ifndef SIDEBRUSH_CONTROLLER_H
#define SIDEBRUSH_CONTROLLER_H

#include <ros/ros.h>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <bumperbot_controller/SetSideBrushSpeed.h>
#include <bumperbot_controller/GetSideBrushSpeed.h>
#include <bumperbot_controller/SetSideBrushPosition.h>
#include <bumperbot_controller/GetSideBrushPosition.h>
#include <bumperbot_controller/SideBrushSpeedFeedback.h>
#include <bumperbot_controller/SideBrushPosFeedback.h>

class SideBrushController
{
public:
    SideBrushController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher sidebrush_speed_pub_;
    ros::Publisher sidebrush_position_pub_;
    ros::ServiceServer set_sidebrush_speed_service_;
    ros::ServiceServer get_sidebrush_speed_service_;
    ros::ServiceServer set_sidebrush_position_service_;
    ros::ServiceServer get_sidebrush_position_service_;

    float current_speed_;
    std::string current_position_;

    // Callback function for the service to set Sidebrush Speed
    bool setSideBrushSpeed(bumperbot_controller::SetSideBrushSpeed::Request& req, 
                        bumperbot_controller::SetSideBrushSpeed::Response& res);

    bool getSideBrushSpeed(bumperbot_controller::GetSideBrushSpeed::Request& req, 
                        bumperbot_controller::GetSideBrushSpeed::Response& res);

    bool setSideBrushPosition(bumperbot_controller::SetSideBrushPosition::Request& req, 
                        bumperbot_controller::SetSideBrushPosition::Response& res);

    bool getSideBrushPosition(bumperbot_controller::GetSideBrushPosition::Request& req,   
                        bumperbot_controller::GetSideBrushPosition::Response& res);
};

#endif 
