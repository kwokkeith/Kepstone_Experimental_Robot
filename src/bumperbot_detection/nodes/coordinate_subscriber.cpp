#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"


void msgCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 3) {
        float x = msg->data[0];
        float y = msg->data[1];
        float z = msg->data[2];

        ROS_INFO("Object Detected at: X=%f, Y=%f, Z=%f", x, y, z);
    } else {
        ROS_WARN("Received unexpected coordinate size: %ld", msg->data.size());
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "coordinate_subscriber");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/object_coordinates", 10, msgCallback);
    ros::spin();

    return 0;

}