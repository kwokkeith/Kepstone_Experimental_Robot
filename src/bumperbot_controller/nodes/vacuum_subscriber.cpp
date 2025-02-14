#include <ros/ros.h>
#include <bumperbot_controller/VacuumPowerFeedback.h>
#include <bumperbot_controller/DustBagCapacityFeedback.h>

void powerCallback(const bumperbot_controller::VacuumPowerFeedback::ConstPtr& msg)
{
    float power = msg->power;
    std::string message = msg->message;
    ROS_INFO("Rollerbrush speed: %f. Message: %s", power, message.c_str());
}

void dustBagCallback(const bumperbot_controller::DustBagCapacityFeedback::ConstPtr& msg)
{
    bool capacity = msg->full;
    ROS_INFO("Dustbag is full: %s.", capacity ? "true" : "false");
}


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "vacuum_subscriber");
    ros::NodeHandle nh_;

    // Load global parameters for services/topics
    std::string vacuum_power_feedback_topic_sub_, dustbag_capacity_feedback_topic_sub_;
    nh_.getParam("/vacuum_controller/topics/vacuum_power_feedback", vacuum_power_feedback_topic_sub_);
    nh_.getParam("/vacuum_controller/topics/dustbag_capacity_feedback", dustbag_capacity_feedback_topic_sub_);

    ros::Subscriber power_sub = nh_.subscribe(vacuum_power_feedback_topic_sub_, 10, powerCallback);
    ros::Subscriber dustbag_sub = nh_.subscribe(dustbag_capacity_feedback_topic_sub_, 10, dustBagCallback);
    ros::spin();

    return 0;

}