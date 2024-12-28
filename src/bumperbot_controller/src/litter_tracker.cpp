#include "bumperbot_controller/litter_tracker.h"

LitterTracker::LitterTracker(ros::NodeHandle& nh) : nh_(nh)
{
    // Initialize service clients
    std::string get_robot_mode_srv_client_, get_next_target_litter_srv_client_;
    std::string mode_switch_request_srv_client_, get_amcl_pose_srv_client_, has_litter_to_clear_srv_client_;
    nh_.getParam("/robot_controller/services/get_current_mode", get_robot_mode_srv_client_);
    nh_.getParam("/litter_manager/services/next_waypoint", get_next_target_litter_srv_client_);
    nh_.getParam("/robot_controller/services/mode_switch", mode_switch_request_srv_client_);
    nh_.getParam("/navigation/services/get_amcl_pose", get_amcl_pose_srv_client_);
    nh_.getParam("/litter_manager/services/has_litter_to_clear", has_litter_to_clear_srv_client_);

    get_robot_mode_client_ = nh_.serviceClient<bumperbot_controller::GetCurrentMode>(get_robot_mode_srv_client_);
    get_next_target_litter_client_ = nh_.serviceClient<litter_destruction::GetNextTargetLitter>(get_next_target_litter_srv_client_);
    mode_switch_request_client_ = nh_.serviceClient<bumperbot_controller::ModeSwitch>(mode_switch_request_srv_client_);
    get_amcl_pose_client_ = nh_.serviceClient<navigation::GetAmclPose>(get_amcl_pose_srv_client_);
    has_litter_to_clear_client_ = nh_.serviceClient<litter_destruction::HasLitterToClear>(has_litter_to_clear_srv_client_);

    // Initialize distance threshold
    if (!nh_.getParam("/litter_tracker/distance_threshold", distance_threshold_))
    {
        ROS_WARN("Parameter '/litter_tracker/distance_threshold' not found. Using default value 0.5.");
        distance_threshold_ = 0.8; // Default value
    }
}

void LitterTracker::spin()
{
    ros::Rate rate(1.0); // Check at 1 Hz

    while (ros::ok())
    {
        // Check robot mode
        bumperbot_controller::GetCurrentMode robot_mode_srv;
        if (!get_robot_mode_client_.call(robot_mode_srv))
        {
            ROS_WARN("Failed to call service to get robot mode.");
            rate.sleep();
            continue;
        }

        if (static_cast<int>(robot_mode_srv.response.mode) != static_cast<int>(RobotMode::LITTER_PICKING))
        {
            ROS_INFO("Robot is not in LITTER_PICKING mode. Skipping check.");
            rate.sleep();
            continue;
        }

        // Check if there is any more litter to clear
        litter_destruction::HasLitterToClear has_litter_to_clear_srv;
        if (!has_litter_to_clear_client_.call(has_litter_to_clear_srv))
        {
            ROS_WARN("Failed to call service to check HasLitterToClear");
            continue; // Exit if the service call fails
        }

        // If no more litter to clear then ignore o.w. handle the litter
        if (!has_litter_to_clear_srv.response.has_litter){
            continue;
        }

        // Get current robot position
        navigation::GetAmclPose amcl_pose_srv;
        if (!get_amcl_pose_client_.call(amcl_pose_srv))
        {
            ROS_WARN("Failed to call service to get robot AMCL pose.");
            rate.sleep();
            continue;
        }
        geometry_msgs::Point current_position = amcl_pose_srv.response.pose.pose.pose.position;

        // Get current target litter
        litter_destruction::GetNextTargetLitter next_target_srv;
        if (!get_next_target_litter_client_.call(next_target_srv))
        {
            ROS_WARN("Failed to call service to get next target litter.");
            rate.sleep();
            continue;
        }

        if (!next_target_srv.response.success)
        {
            ROS_WARN("No valid target litter found. Skipping check.");
            rate.sleep();
            continue;
        }

        geometry_msgs::Point target_position = next_target_srv.response.litter.point;

        // Calculate distance to target litter
        double distance = std::sqrt(std::pow(target_position.x - current_position.x, 2) +
                                    std::pow(target_position.y - current_position.y, 2));

        ROS_INFO_STREAM("Distance to target litter: " << distance << " meters. Distance threshold: " << distance_threshold_);

        // If distance is below the threshold, switch mode
        if (distance <= distance_threshold_)
        {
            bumperbot_controller::ModeSwitch mode_switch_srv;
            mode_switch_srv.request.mode = int(RobotMode::LITTER_TRACKING);
            if (!mode_switch_request_client_.call(mode_switch_srv))
            {
                ROS_WARN("Failed to call service to switch robot mode.");
            }
            else if (!mode_switch_srv.response.success)
            {
                ROS_WARN("Mode switch service returned failure.");
            }
            else
            {
                ROS_INFO("Successfully switched robot mode.");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
}
