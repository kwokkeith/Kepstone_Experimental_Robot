#ifndef LITTER_TRACKER_H
#define LITTER_TRACKER_H
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "navigation/GetAmclPose.h"
#include "bumperbot_controller/ModeSwitch.h"
#include "bumperbot_controller/GetCurrentMode.h"
#include "litter_destruction/GetNextTargetLitter.h"
#include <litter_destruction/HasLitterToClear.h>

enum class RobotMode {
    IDLE            = 1,
    COVERAGE        = 2,
    LITTER_PICKING  = 3,
    TRANSITION      = 4,
    LITTER_TRACKING = 5
};

class LitterTracker
{
public:
    LitterTracker(ros::NodeHandle& nh);
    void spin();

private:
    ros::NodeHandle nh_;    
    double distance_threshold_;                         // Distance threshold from litter to go into Tracking Mode
    ros::ServiceClient get_robot_mode_client_;          // Service to get robot mode
    ros::ServiceClient get_next_target_litter_client_;  // Service to get next target litter from litter manager
    ros::ServiceClient mode_switch_request_client_;     // Service to change robot mode
    ros::ServiceClient get_amcl_pose_client_;           // Service to get amcl position of robot
    ros::ServiceClient has_litter_to_clear_client_;     // Service to check if there is anymore litter to clear

    void checkAndSwitchMode();                          // Check conditions and handle mode switch
};  
#endif