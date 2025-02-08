#include "bumperbot_controller/rollerbrush_controller.h"

RollerBrushController::RollerBrushController(ros::NodeHandle& nh) : 
    nh_(nh), 
    current_power_(0.0),
    current_direction_("up")
{
    // GET global parameters for services/topics
    // Service Servers
    std::string set_rollerbrush_power_srv_svr_, get_rollerbrush_power_srv_svr_;
    std::string set_rollerbrush_direction_srv_svr_, get_rollerbrush_direction_srv_svr_;
    nh_.getParam("/rollerbrush_controller/services/get_rollerbrush_power", get_rollerbrush_power_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_power", set_rollerbrush_power_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/get_rollerbrush_direction", get_rollerbrush_direction_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_direction", set_rollerbrush_direction_srv_svr_);
    // Topic Publishers
    std::string rollerbrush_power_topic_pub_;
    std::string rollerbrush_direction_topic_pub_;
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_power", rollerbrush_power_topic_pub_);
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_direction", rollerbrush_direction_topic_pub_);

    // Initialize Service Servers
    set_rollerbrush_power_service_ = nh_.advertiseService(set_rollerbrush_power_srv_svr_, &RollerBrushController::setRollerBrushPower, this);
    get_rollerbrush_power_service_ = nh_.advertiseService(get_rollerbrush_power_srv_svr_, &RollerBrushController::getRollerBrushPower, this);
    set_rollerbrush_direction_service_ = nh_.advertiseService(set_rollerbrush_direction_srv_svr_, &RollerBrushController::setRollerBrushDirection, this);
    get_rollerbrush_direction_service_ = nh_.advertiseService(get_rollerbrush_direction_srv_svr_, &RollerBrushController::getRollerBrushDirection, this);
    // Initialize Topic Publishers
    rollerbrush_power_pub_ = nh_.advertise<std_msgs::Float32>(rollerbrush_power_topic_pub_, 10);
    rollerbrush_direction_pub_ = nh_.advertise<std_msgs::String>(rollerbrush_direction_topic_pub_, 10);


    // Publish default rollerbrush power (0.0)
    std_msgs::Float32 msg;
    msg.data = current_power_;
    rollerbrush_power_pub_.publish(msg);

    // Publish default rollerbrush direction (up)
    std_msgs::String msg2;
    msg2.data = current_direction_;
    rollerbrush_direction_pub_.publish(msg2);
    

    ROS_INFO("RollerBrush Controller Node Initialized.");
}

bool RollerBrushController::setRollerBrushPower(bumperbot_controller::SetRollerBrushPower::Request& req, 
                                      bumperbot_controller::SetRollerBrushPower::Response& res)
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
        rollerbrush_power_pub_.publish(msg);

        res.success = true;
        res.message = "RollerBrush power set successfully to " + std::to_string(current_power_);
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setRollerBrushPower: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setRollerBrushPower");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool RollerBrushController::getRollerBrushPower(bumperbot_controller::GetRollerBrushPower::Request& req, 
                                      bumperbot_controller::GetRollerBrushPower::Response& res)
{
    res.power = current_power_;
    ROS_INFO_STREAM("Returning current rollerbrush power: " << current_power_);
    return true;
}

bool RollerBrushController::setRollerBrushDirection(bumperbot_controller::SetRollerBrushDirection::Request& req, 
                                      bumperbot_controller::SetRollerBrushDirection::Response& res)
{
    try 
    {
        // Validate input
        if (req.direction != "up" && req.direction != "down")
        {
            res.success = false;
            res.message = "Invalid direction! Please set a value of 'up' or 'down'.";
            ROS_WARN_STREAM(res.message);
            return false;
        }

        // Set new direction
        current_direction_ = req.direction;
        std_msgs::String msg;
        msg.data = current_direction_;
        rollerbrush_direction_pub_.publish(msg);

        res.success = true;
        res.message = "RollerBrush direction set successfully to " + current_direction_;
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setRollerBrushDirection: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setRollerBrushDirection");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool RollerBrushController::getRollerBrushDirection(bumperbot_controller::GetRollerBrushDirection::Request& req, 
                                      bumperbot_controller::GetRollerBrushDirection::Response& res)
{
    res.direction = current_direction_;
    ROS_INFO_STREAM("Returning current rollerbrush direction: " << current_direction_);
    return true;
}