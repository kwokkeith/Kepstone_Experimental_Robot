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
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera_frame/detected_object_coordinates", 10, msgCallback);
    ros::spin();

    return 0;

}