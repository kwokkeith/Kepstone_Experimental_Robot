#include "litter_destruction/boundary_visualizer.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>

BoundaryVisualizer::BoundaryVisualizer(ros::NodeHandle &nh)
{
    // Get Global Parameter for service/topics
    std::string global_boundary_marker_topic_pub_;
    std::string local_boundary_marker_topic_pub_;
    std::string republish_global_boundary_svc_srv_;
    std::string republish_local_boundary_svc_srv_;
    nh.getParam("/boundary_visualizer/topics/global_boundary_marker", global_boundary_marker_topic_pub_);
    nh.getParam("/boundary_visualizer/topics/local_boundary_marker", local_boundary_marker_topic_pub_);
    nh.getParam("/boundary_visualizer/services/republish_global_boundary", republish_global_boundary_svc_srv_);
    nh.getParam("/boundary_visualizer/services/republish_local_boundary", republish_local_boundary_svc_srv_);
    std::string get_global_boundary_client_;
    std::string get_local_boundary_client_;
    std::string get_current_mode_client_;
    nh.getParam("/robot_controller/services/get_global_boundary", get_global_boundary_client_);
    nh.getParam("/robot_controller/services/get_current_mode", get_current_mode_client_);
    nh.getParam("/litter_manager/services/get_local_boundary_center", get_local_boundary_client_);


    // Initialize publishers for the boundary markers
    global_boundary_marker_pub_ = nh.advertise<visualization_msgs::Marker>(global_boundary_marker_topic_pub_, 10);
    local_boundary_marker_pub_  = nh.advertise<visualization_msgs::Marker>(local_boundary_marker_topic_pub_, 10);

    // Initialize service clients to retrieve boundary data
    get_global_boundary_srv_ = nh.serviceClient<litter_destruction::GlobalBoundaryCenter>(get_global_boundary_client_);
    get_local_boundary_srv_  = nh.serviceClient<litter_destruction::LocalBoundaryCenter>(get_local_boundary_client_);
    get_robot_mode_srv_      = nh.serviceClient<bumperbot_controller::GetCurrentMode>(get_current_mode_client_);    

    // Initialize service servers to handle republishing requests
    republish_global_boundary_srv_ = nh.advertiseService(republish_global_boundary_svc_srv_, &BoundaryVisualizer::republishGlobalBoundary, this);
    republish_local_boundary_srv_  = nh.advertiseService(republish_local_boundary_svc_srv_, &BoundaryVisualizer::republishLocalBoundary, this);
}

bool BoundaryVisualizer::republishGlobalBoundary(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    litter_destruction::GlobalBoundaryCenter global_boundary_srv;

    // Check if the current robot mode is in Litter Picking mode
    bumperbot_controller::GetCurrentMode current_mode_srv;
    if (get_robot_mode_srv_.call(current_mode_srv))
    {
        // TODO: change the enum mode state to a label
        if (current_mode_srv.response.mode == 3) {
            if (get_global_boundary_srv_.call(global_boundary_srv) && global_boundary_srv.response.valid)
                {
                    publishGlobalBoundaryMarker(global_boundary_srv.response.center, global_boundary_srv.response.radius);
                    res.success = true;
                    res.message = "Global boundary marker published.";
                }
                else
                {
                    res.success = false;
                    res.message = "Failed to retrieve global boundary data.";
                }
        }
        else {
            publishGlobalBoundaryMarker(geometry_msgs::Point(), 0);
            res.success = true;
            res.message = "Global boundary marker published, removed because not in LITTER_PICKING mode.";
        }
    }
    return true;
}

bool BoundaryVisualizer::republishLocalBoundary(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    litter_destruction::LocalBoundaryCenter local_boundary_srv;

    // Check if the current robot mode is in Litter Picking mode
    bumperbot_controller::GetCurrentMode current_mode_srv;
    if (get_robot_mode_srv_.call(current_mode_srv))
    {
        if (current_mode_srv.response.mode == 3) {
            if (get_local_boundary_srv_.call(local_boundary_srv) && local_boundary_srv.response.valid)
            {
                publishLocalBoundaryMarker(local_boundary_srv.response.center, local_boundary_srv.response.radius);
                res.success = true;
                res.message = "Local boundary marker published.";
            }
            else
            {
                publishLocalBoundaryMarker(local_boundary_srv.response.center, local_boundary_srv.response.radius);
                res.success = true;
                res.message = "Local boundary marker published.";
            }
        }
        else {
            publishLocalBoundaryMarker(geometry_msgs::Point(), 0);
            res.success = true;
            res.message = "Local boundary marker published.";
        }
    }
    return true;
}

void BoundaryVisualizer::publishGlobalBoundaryMarker(const geometry_msgs::Point &center, double radius)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "global_boundary";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    global_boundary_marker_pub_.publish(marker);
}

void BoundaryVisualizer::publishLocalBoundaryMarker(const geometry_msgs::Point &center, double radius)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "local_boundary";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position = center;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2;
    marker.scale.y = radius * 2;
    marker.scale.z = 0.1;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;

    local_boundary_marker_pub_.publish(marker);
}
