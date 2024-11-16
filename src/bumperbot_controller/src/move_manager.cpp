#include "bumperbot_controller/move_manager.h"

MoveManager::MoveManager(ros::NodeHandle& nh) :
        nh_(nh),
        move_base_client_("move_base", true),
        current_mode_(RobotMode::IDLE), // Initial mode of robot (IDLE)
        coverage_complete_(true)
{
    // Initialize subscribers to the robot mode
    robot_mode_sub_ = nh_.subscribe("robot_controller/robot_mode", 10, &MoveManager::robotModeCallback, this);

    // Initialize service clients
    get_next_litter_client_         = nh_.serviceClient<litter_destruction::GetNextLitter>("/litter_manager/get_next_litter");        // Service to update to next litter in litter manager
    get_next_target_litter_client   = nh_.serviceClient<litter_destruction::GetNextTargetLitter>("/litter_manager/next_waypoint");
    delete_litter_client_           = nh_.serviceClient<litter_destruction::RemoveLitter>("/litter_manager/delete_litter");           // Service to delete litter in litter manager
    get_next_waypoint_client_       = nh_.serviceClient<navigation::GetNextWaypoint>("/coverage_path/next_waypoint");                 // Service to get next target litter
    update_waypoint_client_         = nh_.serviceClient<std_srvs::SetBool>("waypoint_manager/get_next_waypoint");                     // Service to update waypoint_manager (get next waypoint)
    has_litter_to_clear_client_     = nh_.serviceClient<litter_destruction::HasLitterToClear>("/litter_manager/has_litter_to_clear"); // Service to check if any more litter to clear (litter manager)
    mode_switch_request_client_     = nh_.serviceClient<bumperbot_controller::ModeSwitch>("/robot_controller/mode_switch");           // Service to change robot mode
    get_global_boundary_client_     = nh_.serviceClient<litter_destruction::GlobalBoundaryCenter>("/robot_controller/get_global_boundary"); // Service to get global boundary center from robot controller

    ROS_INFO("Waiting for move_base action server...");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base action server.");
}

// Callback function for the robot mode
void MoveManager::robotModeCallback(const std_msgs::Int32::ConstPtr& mode_msg)
{   
    // If current mode of the robot is litter mode and it isnt 
    // in this mode yet, then disrupt the current move command
    if (static_cast<RobotMode>(mode_msg->data) == RobotMode::LITTER_PICKING) {
        if (current_mode_ != RobotMode::LITTER_PICKING) {
            move_base_client_.cancelAllGoals(); // Prioritise litter picking
            
            // Set global boundary center to remember it when transitioning back
            litter_destruction::GlobalBoundaryCenter global_boundary_srv;
            if (get_global_boundary_client_.call(global_boundary_srv)) {
                ROS_WARN("Failed to call service /robot_controller/get_global_boundary");
            } 
            global_boundary_center_ = global_boundary_srv.response.center;
        }
    }

    current_mode_ = static_cast<RobotMode>(mode_msg->data);
}

bool MoveManager::navigateToWaypoint(const geometry_msgs::Point& waypoint, double timeout) {
    // Ensure move_base client is available
    if (!move_base_client_.waitForServer(ros::Duration(5.0))) {
        ROS_WARN("move_base action server is not available.");
        return false;
    }

    // Prepare move_base goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position = waypoint;
    goal.target_pose.pose.orientation.w = 1.0; // No rotation

    // Send goal
    move_base_client_.cancelAllGoals();
    ROS_INFO("Sending waypoint to move_base: [%.2f, %.2f]", waypoint.x, waypoint.y);
    move_base_client_.sendGoal(goal);

    // Monitor result
    ros::Time start_time = ros::Time::now();
    while (!move_base_client_.waitForResult(ros::Duration(0.1))) {
        if (ros::Time::now() - start_time > ros::Duration(timeout)) {
            ROS_WARN("Timed out waiting for move_base to reach waypoint.");
            move_base_client_.cancelGoal();
            return false;
        }
    }

    if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Reached waypoint successfully.");
        return true;
    } else {
        ROS_WARN("Failed to reach waypoint.");
        return false;
    }
}


void MoveManager::performLitterMode() {
    ROS_INFO("Performing Litter Picking Mode");

    // Call the GetNextLitter service (to update litter_manager to pop one litter in its memory)
    litter_destruction::GetNextLitter get_next_litter_srv;
    if (!get_next_litter_client_.call(get_next_litter_srv))
    {
        ROS_WARN("Failed to call service /litter_manager/get_next_litter in litter");
        return; // Exit if the service call fails
    }

    // Check if there is any more litter to clear
    litter_destruction::HasLitterToClear has_litter_to_clear_srv;
    if (!has_litter_to_clear_client_.call(has_litter_to_clear_srv))
    {
        ROS_WARN("Failed to call service /litter_manager/has_litter_to_clear");
        return; // Exit if the service call fails
    }

    // If no more litter to clear then change mode o.w. handle the litter
    if (!has_litter_to_clear_srv.response.has_litter){
        ROS_INFO("Returning from LITTER_MODE");
        ModeSwitchRequest(RobotMode::TRANSITION);
        return;
    }

    // Log the start of the litter handling process
    ROS_INFO("Performing single step in litter mode...");

    // Call the GetNextTargetLitter service
    // To get the next TARGET litter
    litter_destruction::GetNextTargetLitter get_next_target_litter_srv;
    if (!get_next_target_litter_client.call(get_next_target_litter_srv))
    {
        ROS_WARN("Failed to call service /litter_manager/get_next_litter in litter");
        return; // Exit if the service call fails
    }

    // Check if a valid next litter point was returned
    if (!get_next_target_litter_srv.response.success)
    {
        ROS_INFO("Failed to get litter waypoint from service.");
        return; // Exit
    }

    // Extract the next litter waypoint
    geometry_msgs::Point litter_point = get_next_target_litter_srv.response.litter.point;

    // Log the received waypoint
    ROS_INFO_STREAM("Received litter waypoint: [" << litter_point.x << ", " << litter_point.y << ", " << litter_point.z << "]");

    // Navigate to the litter waypoint
    if (navigateToWaypoint(litter_point))
    {
        ROS_INFO_STREAM("Successfully reached litter waypoint: [" << litter_point.x << ", " << litter_point.y << ", " << litter_point.z << "]");

        // Simulate litter destruction
        ROS_INFO("Destroying litter...");
        ros::Duration(5.0).sleep(); // Simulate the time taken to destroy the litter

        // Call the service to remove the litter from memory
        litter_destruction::RemoveLitter remove_litter_srv;
        remove_litter_srv.request.litter = get_next_target_litter_srv.response.litter;

        if (!delete_litter_client_.call(remove_litter_srv))
        {
            ROS_ERROR("Failed to call service /litter_manager/delete_litter");
            return;
        }

        if (remove_litter_srv.response.success)
        { ROS_INFO_STREAM("Successfully removed litter with ID: " << get_next_target_litter_srv.response.litter.id); }
        else
        { ROS_WARN_STREAM("Failed to remove litter with ID: " << get_next_target_litter_srv.response.litter.id); }
    }
    else
    { ROS_WARN_STREAM("Failed to reach litter waypoint: [" << litter_point.x << ", " << litter_point.y << ", " << litter_point.z << "]"); }
}


void MoveManager::performCoverageMode() {
    ROS_INFO("Performing single step in coverage mode...");

    coverage_complete_ = false;
    // Call the service to get the next coverage waypoint
    navigation::GetNextWaypoint next_waypoint_srv;
    if (!get_next_waypoint_client_.call(next_waypoint_srv))
    {
        ROS_WARN("Failed to call service /coverage_manager/get_next_waypoint");
        return; // Exit if the service call fails
    }

    // Check if a valid waypoint was returned
    if (!next_waypoint_srv.response.success)
    {
        ROS_INFO("No more coverage points to handle.");
        coverage_complete_ = true; // Mark coverage as complete
        
        // Switch back to IDLE mode
        ModeSwitchRequest(RobotMode::IDLE);
        return;
    }

    // Extract the coverage waypoint
    geometry_msgs::Point coverage_point = next_waypoint_srv.response.point;

    // Log the received waypoint
    ROS_INFO_STREAM("Received coverage waypoint: [" << coverage_point.x << ", " << coverage_point.y << ", " << coverage_point.z << "]");

    // Navigate to the coverage waypoint
    if (navigateToWaypoint(coverage_point))
    {
        ROS_INFO_STREAM("Successfully reached coverage waypoint: [" << coverage_point.x << ", " << coverage_point.y << ", " << coverage_point.z << "]");

        // Mark the current waypoint as completed
        std_srvs::SetBool bool_srvs;
        bool_srvs.request.data = true;
        if (!update_waypoint_client_.call(bool_srvs))
        { ROS_WARN("Failed to call service /coverage_manager/update_waypoint_status."); }
    }
    else
    { ROS_WARN_STREAM("Failed to reach coverage waypoint: [" << coverage_point.x << ", " << coverage_point.y << ", " << coverage_point.z << "]"); }
}


void MoveManager::performTransition() {
    if (!isCoverageComplete()){
        // Transition to the previous Coverage Mode
        int retry_count = 3;

        for (int count=0; count<3; count++) {
            // Navigate back to global boundary center
            // Get global boundary center data from robot controller
            if (navigateToWaypoint(global_boundary_center_)) {
                ROS_INFO_STREAM("Successfully reached coverage waypoint: [" << global_boundary_center_.x << ", " << global_boundary_center_.y << ", " << global_boundary_center_.z << "]");
                ModeSwitchRequest(RobotMode::COVERAGE);
                return;
            }
            else {
                ROS_INFO_STREAM("Robot failed to return to global center, retrying " << count + 1 << " of " << retry_count);
            }

            // Check if robot is still in transition mode, if it is not then break
            if (current_mode_ != RobotMode::TRANSITION) break;
        }
        ROS_ERROR_STREAM("Robot failed to reach global center after " << retry_count << " tries!");
    }
    // Robot was not in COVERAGE mode previously
    else {
        // TODO: Add other logic for when you transition not to a COVERAGE mode
        ModeSwitchRequest(RobotMode::IDLE);
    }

}

// HELPER Functions
bool MoveManager::isCoverageComplete() {
    return coverage_complete_;
}


bool MoveManager::ModeSwitchRequest(RobotMode req_mode) {
    bumperbot_controller::ModeSwitch mode_switch_request_srv;
    mode_switch_request_srv.request.mode = int(req_mode);
    if (!mode_switch_request_client_.call(mode_switch_request_srv)) {
        ROS_WARN("move_manager: Failed to perform mode switch");
        return false;
    }
    
    if (!mode_switch_request_srv.response.success) {
        ROS_WARN("move_manager: Failed to perform mode switch");
        return false;
    }

    ROS_INFO_STREAM("move_manager: Mode switched to " << static_cast<int>(req_mode));
    return true;
}


void MoveManager::spin() {
    // Frequency of publishing waypoint (if still performing waypoint then ignored)
    ros::Rate rate(1.0); 

    while (ros::ok()) {
        switch (current_mode_) {
            case RobotMode::TRANSITION: 
                performTransition();
                break;
            case RobotMode::LITTER_PICKING: 
                performLitterMode();
                break;
            case RobotMode::COVERAGE: 
                performCoverageMode();
                break;
            case RobotMode::IDLE:
                ROS_INFO("Robot is in IDLE mode.");
                break;
            default:
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}