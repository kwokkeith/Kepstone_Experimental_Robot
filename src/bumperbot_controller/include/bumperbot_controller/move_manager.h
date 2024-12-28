#ifndef MOVE_MANAGER_H
#define MOVE_MANAGER_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <litter_destruction/GetNextLitter.h>
#include <litter_destruction/RemoveLitter.h>
#include <litter_destruction/HasLitterToClear.h>
#include <litter_destruction/GetNextTargetLitter.h>
#include <litter_destruction/GlobalBoundaryCenter.h>
#include <bumperbot_detection/LitterPoint.h>
#include <bumperbot_controller/ModeSwitch.h>
#include <navigation/GetNextWaypoint.h>
#include <navigation/GetAmclPose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

enum class RobotMode {
    IDLE            = 1,
    COVERAGE        = 2,
    LITTER_PICKING  = 3,
    TRANSITION      = 4,
    LITTER_TRACKING = 5
};

class MoveManager
{
public:
    MoveManager(ros::NodeHandle& nh);

    void spin();

private:
    ros::NodeHandle nh_;     
    ros::Subscriber robot_mode_sub_;                    // Subscriber to the Robot Mode Topic
    ros::Subscriber detected_litter_pose_sub_;          // Subscriber to get detected litter positions from the camera
    ros::ServiceClient get_next_litter_client_;         // Service to get next litter from litter manager
    ros::ServiceClient get_next_target_litter_client_;  // Service to get next target litter from litter manager
    ros::ServiceClient delete_litter_client_;           // Service to remove litter from litter manager
    ros::ServiceClient get_next_waypoint_client_;       // Service to get next waypoint from coverage manager
    ros::ServiceClient update_waypoint_client_;         // Service to update the waypoint manager that previous waypoint reached
    ros::ServiceClient has_litter_to_clear_client_;     // Service to check if any more litter to clear
    ros::ServiceClient mode_switch_request_client_;     // Service to change robot mode
    ros::ServiceClient get_global_boundary_client_;     // Service to get global boundary center from robot controller
    ros::ServiceClient get_amcl_pose_client_;           // Service to get current robot's amcl position

    ros::ServiceServer cancel_goals_service_;           // Service server to cancel all move base goals

    bumperbot_detection::LitterPoint target_litter_; // To keep track of current target litter  

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

    RobotMode current_mode_;
    bool coverage_complete_;
    bool nearest_litter_calculated_;
    geometry_msgs::Point nearest_litter_;
    geometry_msgs::Point global_boundary_center_;
    std::string detected_litter_topic_sub_;



    // Callback functions for the subscribers
    void robotModeCallback(const std_msgs::Int32::ConstPtr& mode_msg);
    bool navigateToWaypoint(const geometry_msgs::Point& waypoint, double timeout = 2.0);

    bool cancelGoalsCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void performLitterMode();
    void performCoverageMode();
    void performTransition();
    void performLitterTracking();
    bool removeLitter(const bumperbot_detection::LitterPoint& litter);
    geometry_msgs::Point getNearestDetectedLitter();
    bool isCoverageComplete();
    bool ModeSwitchRequest(RobotMode req_mode);
};
#endif