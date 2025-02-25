#ifndef MISSION_GEN_H
#define MISSION_GEN_H

#include <ros/ros.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <memory>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#define TABLE_LEN 5

namespace mrccc_utils{
namespace mission_gen {

// std::string line_following(int task_id, std::vector<geometry_msgs::PoseStamped> waypoint);
std::string line_following(const int& task_id, 
    const std::vector<geometry_msgs::PoseStamped> &waypoint);

std::string abort(int task_id);

std::string dock(int task_id);

} // namespace mission_gen
} // namespace mrccc_utils
#endif