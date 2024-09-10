#include "bumperbot_examples/tf_examples.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


TfExamples::TfExamples(const ros::NodeHandle &nh) : nh_(nh), x_increment_(0.05), last_x_(0.0)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    static_transform_stamped_.header.stamp = ros::Time::now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";

    static_transform_stamped_.transform.translation.x = 0;
    static_transform_stamped_.transform.translation.y = 0;
    static_transform_stamped_.transform.translation.z = 0.3;

    // With quarternion the below setting represents zero rotation
    static_transform_stamped_.transform.rotation.x = 0;
    static_transform_stamped_.transform.rotation.y = 0;
    static_transform_stamped_.transform.rotation.z = 0;
    static_transform_stamped_.transform.rotation.w = 1;

    static_broadcaster.sendTransform(static_transform_stamped_);
    ROS_INFO_STREAM("Publishing static transform between " << static_transform_stamped_ 
                    << static_transform_stamped_.header.frame_id << " and " << static_transform_stamped_.child_frame_id);

    timer_ = nh_.createTimer(ros::Duration(0.1), &TfExamples::timerCallback, this);
}


void TfExamples::timerCallback(const ros::TimerEvent &)
{
    static tf2_ros::TransformBroadcaster dynamic_broadcaster;
    
    dynamic_transform_stamped_.header.stamp = ros::Time::now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";

    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_; 
    dynamic_transform_stamped_.transform.translation.y = 0;
    dynamic_transform_stamped_.transform.translation.z = 0;

    dynamic_transform_stamped_.transform.rotation.x = 0;
    dynamic_transform_stamped_.transform.rotation.y = 0;
    dynamic_transform_stamped_.transform.rotation.z = 0;
    dynamic_transform_stamped_.transform.rotation.w = 1;

    dynamic_broadcaster.sendTransform(dynamic_transform_stamped_);
    last_x_ = dynamic_transform_stamped_.transform.translation.x;
}