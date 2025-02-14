#include <ros/ros.h>
#include <bumperbot_controller/SideBrushPosFeedback.h>
#include <bumperbot_controller/SideBrushSpeedFeedback.h>

void speedCallback(const bumperbot_controller::SideBrushSpeedFeedback::ConstPtr& msg)
{
    std::uint32_t rpm = msg->speed;
    std::string message = msg->message;
    ROS_INFO("Rollerbrush speed: %u. Message: %s", rpm, message.c_str());
}

void positionCallback(const bumperbot_controller::SideBrushPosFeedback::ConstPtr& msg)
{
    std::string position = msg->position;
    std::string message = msg->message;
    ROS_INFO("Rollerbrush position: %s. Message: %s", position.c_str(), message.c_str());
}   


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "sidebrush_subscriber");
    ros::NodeHandle nh_;

    // Load global parameters for services/topics
    std::string sidebrush_speed_feedback_topic_sub_, sidebrush_position_feedback_topic_sub_;
    nh_.getParam("/sidebrush_controller/topics/sidebrush_speed_feedback", sidebrush_speed_feedback_topic_sub_);
    nh_.getParam("/sidebrush_controller/topics/sidebrush_position_feedback", sidebrush_position_feedback_topic_sub_);

    ros::Subscriber speed_sub = nh_.subscribe(sidebrush_speed_feedback_topic_sub_, 10, speedCallback);
    ros::Subscriber position_sub = nh_.subscribe(sidebrush_position_feedback_topic_sub_, 10, positionCallback);
    ros::spin();

    return 0;

}