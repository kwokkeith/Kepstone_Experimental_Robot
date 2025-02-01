#include "bumperbot_controller/vacuum_controller.h"

VacuumController::VacuumController(ros::NodeHandle& nh) : 
    nh_(nh), 
    current_power_(0.0)
{
    // GET global parameters for services/topics
    // Service Servers
    std::string set_vacuum_power_srv_svr_, get_vacuum_power_srv_svr_;
    nh_.getParam("/vacuum_controller/services/get_vacuum_power", get_vacuum_power_srv_svr_);
    nh_.getParam("/vacuum_controller/services/set_vacuum_power", set_vacuum_power_srv_svr_);
    // Topic Publishers
    std::string vacuum_power_topic_pub_;
    nh_.getParam("/vacuum_controller/topics/vacuum_power", vacuum_power_topic_pub_);

    // Initialize Service Servers
    set_vacuum_service_ = nh_.advertiseService(set_vacuum_power_srv_svr_, &VacuumController::setVacuumPower, this);
    get_vacuum_service_ = nh_.advertiseService(get_vacuum_power_srv_svr_, &VacuumController::getVacuumPower, this);
    // Initialize Topic Publishers
    vacuum_pub_ = nh_.advertise<std_msgs::Float32>(vacuum_power_topic_pub_, 10);


    // Publish default vacuum power (0.0)
    std_msgs::Float32 msg;
    msg.data = current_power_;
    vacuum_pub_.publish(msg);

    ROS_INFO("Vacuum Controller Node Initialized.");
}

bool VacuumController::setVacuumPower(bumperbot_controller::SetVacuumPower::Request& req, 
                                      bumperbot_controller::SetVacuumPower::Response& res)
{
    try 
    {
        // Validate input range
        if (req.power < 0.0 || req.power > 1.0)
        {
            res.success = false;
            res.message = "Invalid power level! Please set a value between 0.0 and 1.0 (inclusive).";
            ROS_WARN_STREAM(res.message);
            return false;
        }

        // Set new power level
        current_power_ = req.power;
        std_msgs::Float32 msg;
        msg.data = current_power_;
        vacuum_pub_.publish(msg);

        res.success = true;
        res.message = "Vacuum power set successfully to " + std::to_string(current_power_);
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setVacuumPower: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setVacuumPower");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool VacuumController::getVacuumPower(bumperbot_controller::GetVacuumPower::Request& req, 
                                      bumperbot_controller::GetVacuumPower::Response& res)
{
    res.power = current_power_;
    ROS_INFO_STREAM("Returning current vacuum power: " << current_power_);
    return true;
}