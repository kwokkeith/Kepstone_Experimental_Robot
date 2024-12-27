#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
// This node is needed to republish for use with ros localization package

ros::Publisher imu_pub;


void imuCallback(const sensor_msgs::Imu &imu)
{
    sensor_msgs::Imu new_imu;
    new_imu = imu;
    new_imu.header.frame_id = "base_footprint_ekf";
    imu_pub.publish(new_imu);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_republisher_node");
    ros::NodeHandle nh;

    // Load global parameters for services/topics
    std::string imu_ekf_topic_pub_;
    nh.getParam("/localization/topics/imu_ekf", imu_ekf_topic_pub_);
    std::string imu_topic_sub_;
    nh.getParam("/localization/topics/imu", imu_topic_sub_);

    imu_pub = nh.advertise<sensor_msgs::Imu>(imu_ekf_topic_pub_, 10);
    ros::Subscriber imu_sub = nh.subscribe(imu_topic_sub_, 1000, imuCallback);

    ros::spin();
    return 0;
};