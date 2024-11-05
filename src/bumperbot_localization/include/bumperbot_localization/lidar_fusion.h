#ifndef LIDAR_FUSION_H
#define LIDAR_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class LiDARFusion
{
public:
    LiDARFusion();

private:
    ros::NodeHandle nh_;
    ros::Publisher fused_pub_;

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_3d_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_left_;
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_right_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync_;

    void combineScans(const sensor_msgs::LaserScan::ConstPtr& scan_3d,
                      const sensor_msgs::LaserScan::ConstPtr& scan_left,
                      const sensor_msgs::LaserScan::ConstPtr& scan_right);
};

#endif
