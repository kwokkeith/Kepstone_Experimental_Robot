#ifndef VACUUM_CONTROLLER_H
#define VACUUM_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <bumperbot_controller/SetVacuumPower.h>
#include <bumperbot_controller/GetVacuumPower.h>

class VacuumController
{
public:
    VacuumController(ros::NodeHandle& nh);
    
private:
    ros::NodeHandle nh_;
    ros::Publisher vacuum_pub_;
    ros::ServiceServer set_vacuum_service_;
    ros::ServiceServer get_vacuum_service_;

    float current_power_;

    // Callback function for the service to set Vacuum Power
    bool setVacuumPower(bumperbot_controller::SetVacuumPower::Request& req, 
                        bumperbot_controller::SetVacuumPower::Response& res);

    bool getVacuumPower(bumperbot_controller::GetVacuumPower::Request& req, 
                        bumperbot_controller::GetVacuumPower::Response& res);
};

#endif 
