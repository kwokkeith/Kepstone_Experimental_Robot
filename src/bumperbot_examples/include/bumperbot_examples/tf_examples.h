#ifndef TF_EXAMPLES_H
#define TF_EXAMPLES_H
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class TfExamples
{
public:
    TfExamples(const ros::NodeHandle &);

private:
    ros::NodeHandle nh_;
    geometry_msgs::TransformStamped static_transform_stamped_;
    geometry_msgs::TransformStamped dynamic_transform_stamped_;
    ros::Timer timer_;
    double x_increment_;
    double last_x_;
    

    void timerCallback(const ros::TimerEvent &);

};
#endif