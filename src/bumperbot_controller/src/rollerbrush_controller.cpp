#include "bumperbot_controller/rollerbrush_controller.h"

RollerBrushController::RollerBrushController(ros::NodeHandle& nh) : 
    nh_(nh), 
    current_power_(0.0),
    current_position_("up")
{
    // GET global parameters for services/topics
    // Service Servers
    std::string set_rollerbrush_power_srv_svr_, get_rollerbrush_power_srv_svr_;
    std::string set_rollerbrush_position_srv_svr_, get_rollerbrush_position_srv_svr_;
    nh_.getParam("/rollerbrush_controller/services/get_rollerbrush_power", get_rollerbrush_power_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_power", set_rollerbrush_power_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/get_rollerbrush_position", get_rollerbrush_position_srv_svr_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_position", set_rollerbrush_position_srv_svr_);
    // Topic Publishers
    std::string rollerbrush_power_topic_pub_;
    std::string rollerbrush_position_topic_pub_;
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_power", rollerbrush_power_topic_pub_);
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_position", rollerbrush_position_topic_pub_);

    // Initialize Service Servers
    set_rollerbrush_power_service_ = nh_.advertiseService(set_rollerbrush_power_srv_svr_, &RollerBrushController::setRollerBrushPower, this);
    get_rollerbrush_power_service_ = nh_.advertiseService(get_rollerbrush_power_srv_svr_, &RollerBrushController::getRollerBrushPower, this);
    set_rollerbrush_position_service_ = nh_.advertiseService(set_rollerbrush_position_srv_svr_, &RollerBrushController::setRollerBrushPosition, this);
    get_rollerbrush_position_service_ = nh_.advertiseService(get_rollerbrush_position_srv_svr_, &RollerBrushController::getRollerBrushPosition, this);
    // Initialize Topic Publishers
    rollerbrush_power_pub_ = nh_.advertise<std_msgs::Float32>(rollerbrush_power_topic_pub_, 10);
    rollerbrush_position_pub_ = nh_.advertise<std_msgs::String>(rollerbrush_position_topic_pub_, 10);


    // Publish default rollerbrush power (0.0)
    std_msgs::Float32 msg;
    msg.data = current_power_;
    rollerbrush_power_pub_.publish(msg);

    // Publish default rollerbrush position (up)
    std_msgs::String msg2;
    msg2.data = current_position_;
    rollerbrush_position_pub_.publish(msg2);
    

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

bool RollerBrushController::setRollerBrushPosition(bumperbot_controller::SetRollerBrushPosition::Request& req, 
                                      bumperbot_controller::SetRollerBrushPosition::Response& res)
{
    try 
    {
        // Validate input
        if (req.position != "up" && req.position != "down")
        {
            res.success = false;
            res.message = "Invalid position! Please set a value of 'up' or 'down'.";
            ROS_WARN_STREAM(res.message);
            return false;
        }

        // Set new position
        current_position_ = req.position;
        std_msgs::String msg;
        msg.data = current_position_;
        rollerbrush_position_pub_.publish(msg);

        res.success = true;
        res.message = "RollerBrush position set successfully to " + current_position_;
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setRollerBrushPosition: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setRollerBrushPosition");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool RollerBrushController::getRollerBrushPosition(bumperbot_controller::GetRollerBrushPosition::Request& req, 
                                      bumperbot_controller::GetRollerBrushPosition::Response& res)
{
    res.position = current_position_;
    ROS_INFO_STREAM("Returning current rollerbrush position: " << current_position_);
    return true;
}