#include "i2r_adapter/feedback_parser.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace mrccc_utils {
namespace feedback_parser {

Json::Value string_to_json_parser(const std::string& text) {
    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string err;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(text.c_str(), text.c_str() + text.length(), &root, &err)) {
        ROS_ERROR("Error parsing JSON feedback: %s", err.c_str());
    }
    return root;
}

// Enum for feedback status
enum StatusType {
    kStatusAMCLPose = 0,
    kStatusUnknown = 254
} status_id;

// Convert JSON AMCL pose feedback to ROS PoseStamped
geometry_msgs::PoseStamped json_amclpose_to_pose(const Json::Value& obj) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "map";
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.position.x = obj["payload"]["pose"]["pose"]["position"]["x"].asFloat();
    pose_msg.pose.position.y = obj["payload"]["pose"]["pose"]["position"]["y"].asFloat();
    pose_msg.pose.position.z = obj["payload"]["pose"]["pose"]["position"]["z"].asFloat();

    tf2::Quaternion q;
    q.setX(obj["payload"]["pose"]["pose"]["orientation"]["x"].asFloat());
    q.setY(obj["payload"]["pose"]["pose"]["orientation"]["y"].asFloat());
    q.setZ(obj["payload"]["pose"]["pose"]["orientation"]["z"].asFloat());
    q.setW(obj["payload"]["pose"]["pose"]["orientation"]["w"].asFloat());
    pose_msg.pose.orientation = tf2::toMsg(q);

    return pose_msg;
}

void RobotStateUpdate(const std::string& str, geometry_msgs::PoseStamped& pose_msg) {
    Json::Value obj = string_to_json_parser(str);
    status_id = static_cast<StatusType>(obj["header"]["status_id"].asInt());

    switch (status_id) {
    case kStatusAMCLPose:
        pose_msg = json_amclpose_to_pose(obj);
        ROS_INFO("Updated AMCL Pose from feedback.");
        break;
    case kStatusUnknown:
        ROS_WARN("Unknown feedback status received.");
        break;
    default:
        ROS_WARN("Unhandled feedback type.");
        break;
    }
}

} // namespace feedback_parser
} // namespace mrccc_utils
