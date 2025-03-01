#ifndef LITTER_COORDINATE_TRANSFORMER_H
#define LITTER_COORDINATE_TRANSFORMER_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

// This class handles the litter coordinate transformation from camera frame to base frame
class LitterCoordinateTransformer
{
public:
    LitterCoordinateTransformer(const ros::NodeHandle &, const std::string base_frame, const std::string camera_frame, const std::string rear_camera_frame);
    ~LitterCoordinateTransformer();

private:
    ros::NodeHandle nh_;
    ros::Subscriber litter_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    ros::Subscriber litter_coord_camera_frame_sub_;
    ros::Subscriber litter_coord_rear_camera_frame_sub_;
    ros::Publisher litter_coord_base_frame_pub_;
    std::string base_frame_;
    std::string camera_frame_;
    std::string rear_camera_frame_;

    void litterCoordinatesCallback(const geometry_msgs::PointStamped::ConstPtr&);
    void litterCoordinatesCallback2(const geometry_msgs::PointStamped::ConstPtr&);
};
#endif