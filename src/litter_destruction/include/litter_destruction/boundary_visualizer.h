#ifndef BOUNDARY_VISUALIZER_H
#define BOUNDARY_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include "litter_destruction/GlobalBoundaryCenter.h"
#include "litter_destruction/LocalBoundaryCenter.h"  
#include "bumperbot_controller/GetCurrentMode.h"

class BoundaryVisualizer
{
public:
    BoundaryVisualizer(ros::NodeHandle &nh);

private:
    // Publishers for the boundary markers
    ros::Publisher global_boundary_marker_pub_;
    ros::Publisher local_boundary_marker_pub_;

    // Service clients for retrieving boundary data
    ros::ServiceClient get_global_boundary_srv_;
    ros::ServiceClient get_local_boundary_srv_;
    ros::ServiceClient get_robot_mode_srv_;    

    // Service servers for republishing boundary markers
    ros::ServiceServer republish_global_boundary_srv_;
    ros::ServiceServer republish_local_boundary_srv_;



    // Service handlers for republishing markers
    bool republishGlobalBoundary(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool republishLocalBoundary(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    // Helper functions for publishing markers
    void publishGlobalBoundaryMarker(const geometry_msgs::Point &center, double radius);
    void publishLocalBoundaryMarker(const geometry_msgs::Point &center, double radius);
};

#endif 
