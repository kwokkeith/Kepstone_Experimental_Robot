#include "bumperbot_detection/litter_plotter.h"


LitterPlotter::LitterPlotter()
{
    ros::NodeHandle pnh_("~");
    
    // Load marker configuration directly from parameters
    if (!pnh_.getParam("marker_namespace", marker_namespace_))
        ROS_WARN("Failed to load marker_namespace, using default.");
    if (!pnh_.getParam("marker_scale/x", marker_scale_.x))
        ROS_WARN("Failed to load marker_scale/x, using default.");
    if (!pnh_.getParam("marker_scale/y", marker_scale_.y))
        ROS_WARN("Failed to load marker_scale/y, using default.");
    if (!pnh_.getParam("marker_scale/z", marker_scale_.z))
        ROS_WARN("Failed to load marker_scale/z, using default.");
    if (!pnh_.getParam("marker_color/r", marker_color_.r))
        ROS_WARN("Failed to load marker_color/r, using default.");
    if (!pnh_.getParam("marker_color/g", marker_color_.g))
        ROS_WARN("Failed to load marker_color/g, using default.");
    if (!pnh_.getParam("marker_color/b", marker_color_.b))
        ROS_WARN("Failed to load marker_color/b, using default.");
    if (!pnh_.getParam("marker_color/a", marker_color_.a))
        ROS_WARN("Failed to load marker_color/a, using default.");

    // Subscribe to the litter memory topic
    litter_sub_ = nh_.subscribe("litter_memory", 10, &LitterPlotter::litterCallback, this);
    
    // Publisher for marker to be visualized
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("litter_markers", 10);
}

// Callback to process the litter points and convert them to markers for RVIZ
void LitterPlotter::litterCallback(const bumperbot_detection::LitterList::ConstPtr& msg)
{
    // Create a MarkerArray
    visualization_msgs::MarkerArray marker_array;

    // Create markers for each litter point
    for (size_t i = 0; i < msg->litter_points.size(); ++i)
    {
        const auto& litter_point = msg->litter_points[i];
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = litter_point.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "litter_markers";
        marker.id = litter_point.id;                        // Unique ID for each marker
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = litter_point.point;          // Set the position to the litter point
        marker.pose.position.z = 0.0;                       // Default z to 0
        marker.pose.orientation.w = 1.0;                    // Default orientation
        marker.lifetime = ros::Duration(0);                 // Set 0 to display marker indefinitely

         // Apply scale from marker config
        marker.scale.x = 0.001;
        marker.scale.y = 0.001;
        marker.scale.z = 0.001;

        // Apply color from marker config
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker); // Add the marker to the array
    }

    // Publish the marker array
    marker_pub_.publish(marker_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_litter_plotter");

    LitterPlotter plotter;

    ros::spin();

    return 0;
}
