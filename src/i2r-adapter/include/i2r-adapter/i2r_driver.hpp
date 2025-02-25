#ifndef I2R_DRIVER_H
#define I2R_DRIVER_H

#include <Eigen/Geometry>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

#include "i2r_driver/mission_gen.hpp"

namespace i2r_driver {

std::string send_i2r_line_following_mission(ros::NodeHandle& node, std::string& task_id, 
      const std::vector<geometry_msgs::PoseStamped>& path);

std::string send_i2r_docking_mission(ros::NodeHandle& node, std::string task_id);

tf2::Quaternion get_quat_from_yaw(double yaw);

} // namespace i2r_driver
#endif