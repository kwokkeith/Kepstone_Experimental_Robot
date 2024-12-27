#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"


void msgCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // Extract x, y, z from the PointStamped message
    float x = msg->point.x;
    float y = msg->point.y;
    float z = msg->point.z;

    // Print the received coordinates
    ROS_INFO("Object Detected at: X=%f, Y=%f, Z=%f", x, y, z);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "coordinate_subscriber");
    ros::NodeHandle nh_;

    // Load global parameters for services/topics
    std::string detected_object_coordinates_topic_sub_;
    nh_.getParam("/litter_detection/topics/detected_object_coordinates_camera", detected_object_coordinates_topic_sub_);

    ros::Subscriber sub = nh_.subscribe(detected_object_coordinates_topic_sub_, 10, msgCallback);
    ros::spin();

    return 0;

}