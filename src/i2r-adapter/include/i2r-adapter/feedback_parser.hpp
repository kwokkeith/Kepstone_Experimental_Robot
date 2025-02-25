#ifndef FEEDBACK_PARSER_H
#define FEEDBACK_PARSER_H

#include <jsoncpp/json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>

namespace mrccc_utils {
namespace feedback_parser {

Json::Value string_to_json_parser(const std::string& text);

// Enum for feedback status
enum StatusType {
    kStatusAMCLPose = 0,
    kStatusUnknown = 254
};

// Convert JSON AMCL pose feedback to ROS PoseStamped
geometry_msgs::PoseStamped json_amclpose_to_pose(const Json::Value& obj);

// Update robot state based on JSON feedback
void RobotStateUpdate(const std::string& str, geometry_msgs::PoseStamped& pose_msg);

} // namespace feedback_parser
} // namespace mrccc_utils

#endif // FEEDBACK_PARSER_H
