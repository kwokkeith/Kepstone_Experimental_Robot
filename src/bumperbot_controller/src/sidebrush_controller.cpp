#include "bumperbot_controller/sidebrush_controller.h"

SideBrushController::SideBrushController(ros::NodeHandle& nh) : 
    nh_(nh), 
    current_speed_(0),
    current_direction_("up")
{
    // GET global parameters for services/topics
    // Service Servers
    std::string set_sidebrush_speed_srv_svr_, get_sidebrush_speed_srv_svr_;
    std::string set_sidebrush_direction_srv_svr_, get_sidebrush_direction_srv_svr_;
    nh_.getParam("/sidebrush_controller/services/get_sidebrush_speed", get_sidebrush_speed_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_speed", set_sidebrush_speed_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/get_sidebrush_direction", get_sidebrush_direction_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_direction", set_sidebrush_direction_srv_svr_);
    // Topic Publishers
    std::string sidebrush_speed_topic_pub_;
    std::string sidebrush_direction_topic_pub_;
    nh_.getParam("/sidebrush_controller/topics/sidebrush_speed", sidebrush_speed_topic_pub_);
    nh_.getParam("/sidebrush_controller/topics/sidebrush_direction", sidebrush_direction_topic_pub_);

    // Initialize Service Servers
    set_sidebrush_speed_service_ = nh_.advertiseService(set_sidebrush_speed_srv_svr_, &SideBrushController::setSideBrushSpeed, this);
    get_sidebrush_speed_service_ = nh_.advertiseService(get_sidebrush_speed_srv_svr_, &SideBrushController::getSideBrushSpeed, this);
    set_sidebrush_direction_service_ = nh_.advertiseService(set_sidebrush_direction_srv_svr_, &SideBrushController::setSideBrushDirection, this);
    get_sidebrush_direction_service_ = nh_.advertiseService(get_sidebrush_direction_srv_svr_, &SideBrushController::getSideBrushDirection, this);
    // Initialize Topic Publishers
    sidebrush_speed_pub_ = nh_.advertise<std_msgs::UInt32>(sidebrush_speed_topic_pub_, 10);
    sidebrush_direction_pub_ = nh_.advertise<std_msgs::String>(sidebrush_direction_topic_pub_, 10);


    // Publish default sidebrush speed (0)
    std_msgs::UInt32 msg;
    msg.data = current_speed_;
    sidebrush_speed_pub_.publish(msg);

    // Publish default sidebrush direction (up)
    std_msgs::String msg2;
    msg2.data = current_direction_;
    sidebrush_direction_pub_.publish(msg2);
    

    ROS_INFO("SideBrush Controller Node Initialized.");
}

bool SideBrushController::setSideBrushSpeed(bumperbot_controller::SetSideBrushSpeed::Request& req, 
                                      bumperbot_controller::SetSideBrushSpeed::Response& res)
{
    try 
    {
        // Validate input range
        if (req.speed < 0)
        {
            res.success = false;
            res.message = "Invalid speed! Please set a positive value.";
            ROS_WARN_STREAM(res.message);
            return false;
        }

        // Set new speed level
        current_speed_ = req.speed;
        std_msgs::UInt32 msg;
        msg.data = current_speed_;
        sidebrush_speed_pub_.publish(msg);

        res.success = true;
        res.message = "SideBrush speed set successfully to " + std::to_string(current_speed_);
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setSideBrushSpeed: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setSideBrushSpeed");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool SideBrushController::getSideBrushSpeed(bumperbot_controller::GetSideBrushSpeed::Request& req, 
                                      bumperbot_controller::GetSideBrushSpeed::Response& res)
{
    res.speed = current_speed_;
    ROS_INFO_STREAM("Returning current sidebrush speed: " << current_speed_);
    return true;
}

bool SideBrushController::setSideBrushDirection(bumperbot_controller::SetSideBrushDirection::Request& req, 
                                      bumperbot_controller::SetSideBrushDirection::Response& res)
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
        sidebrush_direction_pub_.publish(msg);

        res.success = true;
        res.message = "SideBrush direction set successfully to " + current_direction_;
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setSideBrushDirection: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setSideBrushDirection");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool SideBrushController::getSideBrushDirection(bumperbot_controller::GetSideBrushDirection::Request& req, 
                                      bumperbot_controller::GetSideBrushDirection::Response& res)
{
    res.direction = current_direction_;
    ROS_INFO_STREAM("Returning current sidebrush direction: " << current_direction_);
    return true;
}