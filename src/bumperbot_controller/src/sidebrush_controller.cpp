#include "bumperbot_controller/sidebrush_controller.h"

SideBrushController::SideBrushController(ros::NodeHandle& nh) : 
    nh_(nh), 
    current_speed_(0),
    current_position_("up")
{
    // GET global parameters for services/topics
    // Service Servers
    std::string set_sidebrush_speed_srv_svr_, get_sidebrush_speed_srv_svr_;
    std::string set_sidebrush_position_srv_svr_, get_sidebrush_position_srv_svr_;
    nh_.getParam("/sidebrush_controller/services/get_sidebrush_speed", get_sidebrush_speed_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_speed", set_sidebrush_speed_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/get_sidebrush_position", get_sidebrush_position_srv_svr_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_position", set_sidebrush_position_srv_svr_);
    // Topic Publishers
    std::string sidebrush_speed_topic_pub_;
    std::string sidebrush_position_topic_pub_;
    nh_.getParam("/sidebrush_controller/topics/sidebrush_speed", sidebrush_speed_topic_pub_);
    nh_.getParam("/sidebrush_controller/topics/sidebrush_position", sidebrush_position_topic_pub_);

    // Initialize Service Servers
    set_sidebrush_speed_service_ = nh_.advertiseService(set_sidebrush_speed_srv_svr_, &SideBrushController::setSideBrushSpeed, this);
    get_sidebrush_speed_service_ = nh_.advertiseService(get_sidebrush_speed_srv_svr_, &SideBrushController::getSideBrushSpeed, this);
    set_sidebrush_position_service_ = nh_.advertiseService(set_sidebrush_position_srv_svr_, &SideBrushController::setSideBrushPosition, this);
    get_sidebrush_position_service_ = nh_.advertiseService(get_sidebrush_position_srv_svr_, &SideBrushController::getSideBrushPosition, this);
    // Initialize Topic Publishers
    sidebrush_speed_pub_ = nh_.advertise<std_msgs::UInt32>(sidebrush_speed_topic_pub_, 10);
    sidebrush_position_pub_ = nh_.advertise<std_msgs::String>(sidebrush_position_topic_pub_, 10);


    // Publish default sidebrush speed (0)
    std_msgs::UInt32 msg;
    msg.data = current_speed_;
    sidebrush_speed_pub_.publish(msg);

    // Publish default sidebrush position (up)
    std_msgs::String msg2;
    msg2.data = current_position_;
    sidebrush_position_pub_.publish(msg2);
    

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

bool SideBrushController::setSideBrushPosition(bumperbot_controller::SetSideBrushPosition::Request& req, 
                                      bumperbot_controller::SetSideBrushPosition::Response& res)
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
        sidebrush_position_pub_.publish(msg);

        res.success = true;
        res.message = "SideBrush position set successfully to " + current_position_;
        ROS_INFO_STREAM(res.message);

        return true;
    }
    catch (const std::exception& e) 
    {
        ROS_ERROR_STREAM("Exception in setSideBrushPosition: " << e.what());
        res.success = false;
        res.message = "Internal server error.";
        return false;
    }
    catch (...)
    {
        ROS_ERROR("Unknown exception in setSideBrushPosition");
        res.success = false;
        res.message = "Unknown internal error.";
        return false;
    }
}

bool SideBrushController::getSideBrushPosition(bumperbot_controller::GetSideBrushPosition::Request& req, 
                                      bumperbot_controller::GetSideBrushPosition::Response& res)
{
    res.position = current_position_;
    ROS_INFO_STREAM("Returning current sidebrush position: " << current_position_);
    return true;
}