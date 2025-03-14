#include "navigation/waypoint_manager.h"
#include "navigation/ConvertPixelWaypointsToMap.h"
#include "bumperbot_utils/utils.h"

// Constructor
WaypointManager::WaypointManager(ros::NodeHandle& nh)
    : nh_(nh), current_index_(0) {
        // GET global parameters for services/topics
        std::string next_waypoint_svc_srv_;
        std::string get_next_waypoint_svc_srv_;
        std::string initiate_coverage_path_svc_srv_;
        std::string get_waypoints_svc_srv_;
        nh_.getParam("/coverage_path/services/next_waypoint", next_waypoint_svc_srv_);
        nh_.getParam("/waypoint_manager/services/get_next_waypoint", get_next_waypoint_svc_srv_);
        nh_.getParam("/waypoint_manager/services/initiate_coverage_path", initiate_coverage_path_svc_srv_);
        nh_.getParam("/waypoint_manager/services/get_waypoints", get_waypoints_svc_srv_);


        waypoint_service_ = nh_.advertiseService(next_waypoint_svc_srv_, &WaypointManager::getNextTargetWaypoint, this);
        update_waypoint_service_ = nh_.advertiseService(get_next_waypoint_svc_srv_, &WaypointManager::getNextWaypoint, this);
        initiate_coverage_service_ = nh_.advertiseService(initiate_coverage_path_svc_srv_, &WaypointManager::initiateCoveragePath, this);
        get_waypoints_service_ = nh_.advertiseService(get_waypoints_svc_srv_, &WaypointManager::getWaypoints, this);
    }

// Service to initiate coverage path by loading waypoints from file
bool WaypointManager::initiateCoveragePath(navigation::InitiateCoveragePath::Request& req,
                                           navigation::InitiateCoveragePath::Response& res) {
    std::string waypoints_file = req.waypoints_file;

    // Load waypoints from file
    std::vector<geometry_msgs::Point> map_waypoints = Utils::readWaypointsFromFile(waypoints_file);

    // Convert pixel waypoints to map waypoints using a service call
    ros::ServiceClient client = nh_.serviceClient<navigation::ConvertPixelWaypointsToMap>("convert_pixel_waypoints_to_map_waypoints");
    navigation::ConvertPixelWaypointsToMap srv;
    srv.request.waypoint_file = waypoints_file;

    if (client.call(srv)) {
        ROS_INFO("Successfully converted pixel waypoints to map waypoints.");
        
        waypoints_ = srv.response.map_waypoints;
        current_index_ = 0;  // Reset to the beginning of the waypoints

        res.success = true;
        res.message = "Coverage path initiated successfully.";
    } else {
        ROS_ERROR("Failed to call service to convert pixel waypoints to map waypoints.");
        res.success = false;
        res.message = "Failed to initiate coverage path.";
    }
    return true;
}


// Service to get loaded waypoints from waypoint manager
bool WaypointManager::getWaypoints(navigation::GetWaypoints::Request& req,
                                   navigation::GetWaypoints::Response& res) {
    res.waypoints = waypoints_;
    return true;
}


// Service to get loaded waypoints from waypoint manager
bool WaypointManager::getNextTargetWaypoint(navigation::GetNextWaypoint::Request& req,
                                            navigation::GetNextWaypoint::Response& res) {
    if (current_index_ < waypoints_.size()) {
        ROS_INFO("Returning target waypoint (%.2f, %.2f)", waypoints_[current_index_].x, waypoints_[current_index_].y);
        res.point = waypoints_[current_index_];
        res.success = true;
    }
    else {
        res.point = geometry_msgs::Point(); // Empty Point
        res.success = false;
        ROS_INFO("All waypoints completed.");
    }
    return true;
}


// Mark the current waypoint as completed and publish the next waypoint
bool WaypointManager::getNextWaypoint(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    if (req.data && current_index_ < waypoints_.size()) {
        ROS_INFO("Marking waypoint (%.2f, %.2f) as completed.", waypoints_[current_index_].x, waypoints_[current_index_].y);
        current_index_++;
        res.success = true;
        res.message = "Waypoint marked as completed.";
    } else {
        res.success = false;
        res.message = "No more waypoints to complete or invalid request.";
    }
    return true;
}
