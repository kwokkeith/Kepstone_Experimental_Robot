#include "bumperbot_localization/lidar_fusion.h"
#include <algorithm>
#include <iostream>

LiDARFusion::LiDARFusion() 
    : sync_(MySyncPolicy(10), sub_3d_, sub_left_, sub_right_) {

        // Get Global Parameters for topic/services
        std::string fused_scan_topic_pub_;
        nh_.getParam("/localization/topics/fused_scan", fused_scan_topic_pub_);

        
        // Subscribe to each LiDAR topic
        sub_3d_.subscribe(nh_, "/bumperbot/laser/3d_front/laser_scan", 10);  // 3D Lidar LaserScan topic
        sub_left_.subscribe(nh_, "/bumperbot/laser/2d_left/scan", 10);
        sub_right_.subscribe(nh_, "/bumperbot/laser/2d_right/scan", 10);

        // Publisher for fused scan data
        fused_pub_ = nh_.advertise<sensor_msgs::LaserScan>(fused_scan_topic_pub_, 10);

        // Register the callback for synchronized scans
        sync_.registerCallback(boost::bind(&LiDARFusion::combineScans, this, _1, _2, _3));
}

void LiDARFusion::combineScans(const sensor_msgs::LaserScan::ConstPtr& scan_3d,
                               const sensor_msgs::LaserScan::ConstPtr& scan_left,
                               const sensor_msgs::LaserScan::ConstPtr& scan_right) {
    // Create a fused scan based on the 3D scan data
    sensor_msgs::LaserScan fused_scan = *scan_3d;  // Use the 3D scan as the primary scan

    // Resize the fused scan ranges to match the 3D LiDAR
    fused_scan.ranges.resize(scan_3d->ranges.size());

    // Fuse ranges from each 2D LiDAR scan, taking the minimum distance at each angle
    for (size_t i = 0; i < scan_right->ranges.size(); ++i) {
        // Take minimum valid range from 3D and 2D LiDARs
        if (!std::isinf(scan_right->ranges[i]) && scan_right->ranges[i] > 0.1) {
            fused_scan.ranges[i] = std::min(fused_scan.ranges[i], scan_right->ranges[i]);
        }
    }

    size_t offset = scan_right->ranges.size();
    for (size_t i = 0; i < scan_left->ranges.size(); ++i) {
        if (!std::isinf(scan_left->ranges[i]) && scan_left->ranges[i] > 0.1) {
            fused_scan.ranges[offset + i] = std::min(fused_scan.ranges[offset + i], scan_left->ranges[i]);
        }
    }

    // Incorporate the 3D LiDAR data, taking the minimum distance of the already fused data vs the 3D data
    for (size_t i = 0; i < scan_3d->ranges.size(); ++i) {
        if (!std::isinf(scan_3d->ranges[i]) && scan_3d->ranges[i] > 0.1) {
            // Take minimum valid range from 3D and fused LiDARs
            fused_scan.ranges[i] = std::min(fused_scan.ranges[i], scan_3d->ranges[i]);
        }
    }

    // Publish the fused scan
    fused_pub_.publish(fused_scan);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_fusion");
    LiDARFusion lidarFusion;
    ros::spin();
    return 0;
}
