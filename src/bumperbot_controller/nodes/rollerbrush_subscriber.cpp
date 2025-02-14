#include <ros/ros.h>
#include <bumperbot_controller/RollerBrushPosFeedback.h>
#include <bumperbot_controller/RollerBrushPowerFeedback.h>

void powerCallback(const bumperbot_controller::RollerBrushPowerFeedback::ConstPtr& msg)
{
    float power = msg->power;
    std::string message = msg->message;
    ROS_INFO("Rollerbrush speed: %f. Message: %s", power, message.c_str());
}

void positionCallback(const bumperbot_controller::RollerBrushPosFeedback::ConstPtr& msg)
{
    std::string position = msg->position;
    std::string message = msg->message;
    ROS_INFO("Rollerbrush position: %s. Message: %s", position.c_str(), message.c_str());
}   


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "rollerbrush_subscriber");
    ros::NodeHandle nh_;

    // Load global parameters for services/topics
    std::string rollerbrush_power_feedback_topic_sub_, rollerbrush_position_feedback_topic_sub_;
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_power_feedback", rollerbrush_power_feedback_topic_sub_);
    nh_.getParam("/rollerbrush_controller/topics/rollerbrush_position_feedback", rollerbrush_position_feedback_topic_sub_);

    ros::Subscriber power_sub = nh_.subscribe(rollerbrush_power_feedback_topic_sub_, 10, powerCallback);
    ros::Subscriber position_sub = nh_.subscribe(rollerbrush_position_feedback_topic_sub_, 10, positionCallback);
    ros::spin();

    return 0;

}