#include "bumperbot_controller/move_manager.h"

MoveManager::MoveManager(ros::NodeHandle& nh) :
        nh_(nh),
        move_base_client_("move_base", true),
        current_mode_(RobotMode::IDLE), // Initial mode of robot (IDLE)
        coverage_complete_(true),
        nearest_litter_calculated_(false),
        sidebrush_deploy_speed_(100) // Target RPM of sidebrush when deployed
{
    // GET global parameters for services/topics
    // Topic Subscribers
    std::string robot_mode_topic_sub_;
    nh_.getParam("/robot_controller/topics/robot_mode", robot_mode_topic_sub_);
    nh_.getParam("/litter_memory/topics/detected_litter_raw", detected_litter_topic_sub_);
    // Service Clients 
    std::string get_next_litter_srv_client_;
    std::string get_next_target_litter_srv_client_;
    std::string delete_litter_srv_client_;
    std::string get_next_waypoint_srv_client_;
    std::string update_waypoint_srv_client_;
    std::string has_litter_to_clear_srv_client_;
    std::string mode_switch_request_srv_client_;
    std::string get_global_boundary_srv_client_;
    std::string get_amcl_pose_srv_client_;
    std::string set_vacuum_power_srv_client_;
    std::string set_rollerbrush_power_srv_client_;
    std::string set_rollerbrush_position_srv_client_;
    std::string set_sidebrush_speed_srv_client_;
    std::string set_sidebrush_position_srv_client_;
    nh_.getParam("/litter_manager/services/get_next_litter", get_next_litter_srv_client_);
    nh_.getParam("/litter_manager/services/next_waypoint", get_next_target_litter_srv_client_);
    nh_.getParam("/litter_manager/services/delete_litter", delete_litter_srv_client_);
    nh_.getParam("/coverage_path/services/next_waypoint", get_next_waypoint_srv_client_);
    nh_.getParam("/waypoint_manager/services/get_next_waypoint", update_waypoint_srv_client_);
    nh_.getParam("/litter_manager/services/has_litter_to_clear", has_litter_to_clear_srv_client_);
    nh_.getParam("/robot_controller/services/mode_switch", mode_switch_request_srv_client_);
    nh_.getParam("/robot_controller/services/get_global_boundary", get_global_boundary_srv_client_);
    nh_.getParam("/navigation/services/get_amcl_pose", get_amcl_pose_srv_client_);
    nh_.getParam("/vacuum_controller/services/set_vacuum_power", set_vacuum_power_srv_client_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_power", set_rollerbrush_power_srv_client_);
    nh_.getParam("/rollerbrush_controller/services/set_rollerbrush_position", set_rollerbrush_position_srv_client_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_speed", set_sidebrush_speed_srv_client_);
    nh_.getParam("/sidebrush_controller/services/set_sidebrush_position", set_sidebrush_position_srv_client_);

    // Service Servers
    std::string cancel_goals_svc_svr_;
    nh_.getParam("/move_manager/services/cancel_all_goals", cancel_goals_svc_svr_); 

    // Initialize subscribers to the robot mode
    robot_mode_sub_ = nh_.subscribe(robot_mode_topic_sub_, 10, &MoveManager::robotModeCallback, this);

    // Initialize service clients
    get_next_litter_client_         = nh_.serviceClient<litter_destruction::GetNextLitter>(get_next_litter_srv_client_);                        // Service to update to next litter in litter manager
    get_next_target_litter_client_  = nh_.serviceClient<litter_destruction::GetNextTargetLitter>(get_next_target_litter_srv_client_);           // Service to get next target litter waypoint (not to update)
    delete_litter_client_           = nh_.serviceClient<litter_destruction::RemoveLitter>(delete_litter_srv_client_);                           // Service to delete litter in litter manager
    get_next_waypoint_client_       = nh_.serviceClient<navigation::GetNextWaypoint>(get_next_waypoint_srv_client_);                            // Service to get next waypoint in coverage
    update_waypoint_client_         = nh_.serviceClient<std_srvs::SetBool>(update_waypoint_srv_client_);                                        // Service to update waypoint_manager (get next waypoint)
    has_litter_to_clear_client_     = nh_.serviceClient<litter_destruction::HasLitterToClear>(has_litter_to_clear_srv_client_);                 // Service to check if any more litter to clear (litter manager)
    mode_switch_request_client_     = nh_.serviceClient<bumperbot_controller::ModeSwitch>(mode_switch_request_srv_client_);                     // Service to change robot mode
    get_global_boundary_client_     = nh_.serviceClient<litter_destruction::GlobalBoundaryCenter>(get_global_boundary_srv_client_);             // Service to get global boundary center from robot controller
    get_amcl_pose_client_           = nh_.serviceClient<navigation::GetAmclPose>(get_amcl_pose_srv_client_);                                    // Service to get amcl position of robot
    set_vacuum_power_client_        = nh_.serviceClient<bumperbot_controller::SetVacuumPower>(set_vacuum_power_srv_client_);                    // Service to set the vacuum power
    set_rollerbrush_power_client_   = nh_.serviceClient<bumperbot_controller::SetRollerBrushPower>(set_rollerbrush_power_srv_client_);          // Service to set the rollerbrush power
    set_rollerbrush_position_client_= nh_.serviceClient<bumperbot_controller::SetRollerBrushPosition>(set_rollerbrush_position_srv_client_);    // Service to set rollerbrush position
    set_sidebrush_speed_client_     = nh_.serviceClient<bumperbot_controller::SetSideBrushSpeed>(set_sidebrush_speed_srv_client_);              // Service to set sidebrush speed
    set_sidebrush_position_client_  = nh_.serviceClient<bumperbot_controller::SetSideBrushPosition>(set_sidebrush_position_srv_client_);        // Service to set sidebrush position

    // Initialize service servers
    cancel_goals_service_ = nh_.advertiseService(cancel_goals_svc_svr_, &MoveManager::cancelGoalsCallback, this);

    // Other parameters of the robot
    int temp_value;
    if (nh_.getParam("/sidebrush/deploy_speed", temp_value)) {
        sidebrush_deploy_speed_ = static_cast<uint32_t>(temp_value);
    } else {
        ROS_WARN("Failed to get parameter /sidebrush/deploy_speed. Using default value.");
    }


    ROS_INFO("Waiting for move_base action server...");
    move_base_client_.waitForServer();
    ROS_INFO("Connected to move_base action server.");
}

// Callback function for the robot mode
void MoveManager::robotModeCallback(const std_msgs::Int32::ConstPtr& mode_msg)
{   
    // If current mode of the robot is litter mode and it isnt 
    // in this mode yet, then disrupt the current move command
    if (static_cast<RobotMode>(mode_msg->data) == RobotMode::LITTER_PICKING || 
        static_cast<RobotMode>(mode_msg->data) == RobotMode::LITTER_TRACKING) {
        if (current_mode_ != RobotMode::LITTER_PICKING ||
            current_mode_ != RobotMode::LITTER_TRACKING) {
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
    ROS_INFO("move_manager: current_mode_ = %d", int(current_mode_));
}


bool MoveManager::cancelGoalsCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    try {
        // Cancel all active goals in move_base
        move_base_client_.cancelAllGoals();
        ROS_INFO("MoveManager: Successfully canceled all goals in move_base.");
        
        res.success = true;
        res.message = "All goals successfully canceled.";
    } catch (const std::exception& e) {
        ROS_ERROR("MoveManager: Exception while canceling goals: %s", e.what());
        res.success = false;
        res.message = "Failed to cancel goals.";
    }
    return true;
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
        target_litter_ = bumperbot_detection::LitterPoint();
        ROS_INFO("Returning from LITTER_MODE");
        ModeSwitchRequest(RobotMode::TRANSITION);
        return;
    }

    // Log the start of the litter handling process
    ROS_INFO("Performing single step in litter mode...");

    // Call the GetNextTargetLitter service
    // To get the next TARGET litter
    litter_destruction::GetNextTargetLitter get_next_target_litter_srv;
    if (!get_next_target_litter_client_.call(get_next_target_litter_srv))
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
    target_litter_ = get_next_target_litter_srv.response.litter;
    geometry_msgs::Point litter_point_ = target_litter_.point;

    // Log the received waypoint
    ROS_INFO_STREAM("Received litter waypoint: [" << litter_point_.x << ", " << litter_point_.y << ", " << litter_point_.z << "]");

    // Navigate to the litter waypoint
    if (current_mode_ == RobotMode::LITTER_PICKING && navigateToWaypoint(litter_point_))
    {
        ROS_INFO_STREAM("Successfully reached litter waypoint: [" << litter_point_.x << ", " << litter_point_.y << ", " << litter_point_.z << "]");

        // Litter Destruction Procedure
        // litterDestructionProcedure();

        // // Simulate litter destruction
        // ROS_INFO("Destroying litter...");
        // ros::Duration(5.0).sleep(); // Simulate the time taken to destroy the litter

        // // Call the service to remove the litter from memory
        // if (!removeLitter(target_litter_)) {
        //     ROS_WARN("Failed to remove the current target litter.");
        // } else {
        //     ROS_INFO("Litter successfully removed.");
        // }
    }
    else
    { ROS_WARN_STREAM("Failed to reach litter waypoint: [" << litter_point_.x << ", " << litter_point_.y << ", " << litter_point_.z << "]"); }
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

void MoveManager::performLitterTracking() {
    ROS_INFO("Performing Litter Tracking Mode");
    move_base_client_.cancelAllGoals(); // Prioritise calculation of nearest litter position using camera detection
    if (nearest_litter_calculated_){
        litterDestructionProcedure();
    }

    // Get the nearest detected litter
    try{
        nearest_litter_ = getNearestDetectedLitter();
        nearest_litter_calculated_ = true;
        ROS_INFO_STREAM("Nearest litter: [" << nearest_litter_.x << ", " << nearest_litter_.y << "]");
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("Error in getNearestDetectedLitter: " << e.what());
        // Handle the error
        ModeSwitchRequest(RobotMode::LITTER_PICKING);
        return;
    }
}

void MoveManager::litterDestructionProcedure() {
    ROS_INFO("Destroying litter...");
    SidebrushDeployConfiguration(); // Deploy sidebrush

    // ********************************************************************
    // After initialising the sidebrush head towards the litter
    if(navigateToWaypoint(nearest_litter_)){
        ROS_INFO("Successfully tracked and reached the nearest litter.");

        // Perform litter destruction
        ros::Duration(5.0).sleep(); // Hold for x seconds to ensure litter enters box
        SidebrushRetractConfiguration(); // Retract sidebrush after completing cleaning

        // ros::Duration(5.0).sleep(); // Simulate litter destruction
        // ROS_INFO("Simulating removing of litter using another PROCEDURE");

        // Call the service to remove the litter from memory
        if (!removeLitter(target_litter_)) {
            ROS_WARN("Failed to remove the current target litter.");
        } else {
            ROS_INFO("Litter successfully removed.");
            ModeSwitchRequest(RobotMode::LITTER_PICKING); // Switch back to LITTER_PICKING mode
        }
        nearest_litter_calculated_ = false;
    } else {
        ROS_WARN("Failed to navigate to the nearest detected litter.");
    }
}

// HELPER Functions
void MoveManager::SidebrushDeployConfiguration() {
    // Deploys the sidebrush for litter destruction
    ROS_INFO("Deploying Sidebrush");
    SidebrushPositionRequest("down");
    SidebrushSpeedRequest(sidebrush_deploy_speed_);
}

void MoveManager::SidebrushRetractConfiguration() {
    // Retracts the sidebrush after litter destruction
    ROS_INFO("Retracting Sidebrush");
    SidebrushSpeedRequest(0);
    SidebrushPositionRequest("up");
}

void MoveManager::SidebrushSpeedRequest(uint32_t speed) {
    // SET Sidebrush Speed
    bumperbot_controller::SetSideBrushSpeed set_sidebrush_speed_srv;
    set_sidebrush_speed_srv.request.speed = speed; // RPM
    if(!set_sidebrush_speed_client_.call(set_sidebrush_speed_srv)){
        ROS_WARN("Failed to call service to set sidebrush speed");
        return; // Exit if the service call fails
    }
    // Check if set sidebrush speed client request is successful
    if (!set_sidebrush_speed_srv.response.success)
    {
        ROS_WARN("Failed to set sidebrush speed");
        
        // Set robot to ERROR Mode if sidebrush fails to initiate
        ModeSwitchRequest(RobotMode::ERROR);
        return;
    }
}

void MoveManager::SidebrushPositionRequest(std::string position) {
    std::transform(position.begin(), position.end(), position.begin(), ::toupper);
     // Validate position
     if (position != "UP" && position != "DOWN") {
        ROS_WARN("Invalid sidebrush position: %s. Expected 'UP' or 'DOWN'.", position.c_str());
        return;
    }

    bumperbot_controller::SetSideBrushPosition set_sidebrush_position_srv;
    set_sidebrush_position_srv.request.position = position; // Position of brush
    if (!set_sidebrush_position_client_.call(set_sidebrush_position_srv))
    {
        ROS_WARN("Failed to call service to set sidebrush position");
        return; // Exit if the service call fails
    }

    // Check if set sidebrush position client request is successful
    if (!set_sidebrush_position_srv.response.success)
    {
        ROS_WARN("Failed to set sidebrush position");
        
        // Set robot to ERROR Mode if sidebrush fails to initiate
        ModeSwitchRequest(RobotMode::ERROR);
        return;
    }
}



geometry_msgs::Point MoveManager::getNearestDetectedLitter() {
    // Define a temporary variable to store detected litter positions
    std::vector<geometry_msgs::Point> detected_litters;

    // Create a subscriber to the detected litter topic
    ros::Subscriber sub = nh_.subscribe<geometry_msgs::Point>(
        detected_litter_topic_sub_, 
        10,
        [&detected_litters](const geometry_msgs::Point::ConstPtr& msg) {
            detected_litters.push_back(*msg);
        });

    // Allow the node to process callbacks for a fixed duration
    float processing_duration = 1.5;
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < processing_duration) {
        ros::spinOnce(); // Process incoming messages
    }

    // Check if any litters were detected
    if (detected_litters.empty()) {
        ROS_WARN("No detected litters received within the timeout.");
        // Call the service to remove the litter from memory
        if (!removeLitter(target_litter_)) {
            ROS_WARN("Failed to remove the current target litter.");
        } else {
            ROS_INFO("Litter successfully removed.");
        }
        throw std::runtime_error("No detected litters received within the timeout.");
    }

    // Get current robot position
    navigation::GetAmclPose amcl_pose_srv;
    if (!get_amcl_pose_client_.call(amcl_pose_srv))
    {
        ROS_WARN("Failed to get robot's current position.");
        throw std::runtime_error("Failed to get robot's current position.");
    }
    geometry_msgs::Point current_position = amcl_pose_srv.response.pose.pose.pose.position;

    // Find the nearest litter
    geometry_msgs::Point nearest_litter;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto& litter : detected_litters) {
        // Euclidean distance of robot position to litter
        double distance = std::sqrt(std::pow(current_position.x - litter.x, 2) +
                            std::pow(current_position.y - litter.y, 2));
        if (distance < min_distance) {
            min_distance = distance;
            nearest_litter = litter;
        }
    }

    ROS_INFO_STREAM("Nearest litter found at: [" << nearest_litter.x << ", " << nearest_litter.y << "] with distance: " << min_distance);

    return nearest_litter;
}

bool MoveManager::removeLitter(const bumperbot_detection::LitterPoint& litter) {
    // Call the service to remove the litter from memory
    litter_destruction::RemoveLitter remove_litter_srv;
    remove_litter_srv.request.litter = litter;

    if (!delete_litter_client_.call(remove_litter_srv))
    {
        ROS_ERROR("Failed to call service /litter_manager/delete_litter");
        return false;
    }

    if (remove_litter_srv.response.success)
    { 
        ROS_INFO_STREAM("Successfully removed litter with ID: " << litter.id); 
        return true;
    }
    else
    { 
        ROS_WARN_STREAM("Failed to remove litter with ID: " << litter.id); 
        return false;    
    }
}


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
    ros::Rate rate(1.5); 

    while (ros::ok()) {
        switch (current_mode_) {
            case RobotMode::TRANSITION: 
                performTransition();
                break;
            case RobotMode::LITTER_TRACKING:
                performLitterTracking();
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