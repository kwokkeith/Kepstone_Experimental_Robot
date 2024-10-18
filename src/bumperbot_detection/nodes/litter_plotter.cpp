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
    // marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("litter_markers", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("litter_markers", 10);
}

void LitterPlotter::litterCallback(const bumperbot_detection::LitterList::ConstPtr& msg)
{
    marker_array_.markers.clear();  // Clear previous markers

    for (size_t i = 0; i < msg->litter_points.size(); ++i)
    {
        const auto& litter_point = msg->litter_points[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = litter_point.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = marker_namespace_;
        marker.id = litter_point.id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = litter_point.point;
        marker.pose.position.z = 0.0;  // Set z to 0
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0);

        // Set the scale
        marker.scale.x = marker_scale_.x;
        marker.scale.y = marker_scale_.y;
        marker.scale.z = marker_scale_.z;

        // Set the color
        marker.color.r = marker_color_.r;
        marker.color.g = marker_color_.g;
        marker.color.b = marker_color_.b;
        marker.color.a = marker_color_.a;

        marker_array_.markers.push_back(marker);  // Store markers
    }

    // Publish the marker array immediately
    marker_pub_.publish(marker_array_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_litter_plotter");

    LitterPlotter plotter;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // Publish the stored marker array continuously
        plotter.marker_pub_.publish(plotter.marker_array_);

        ros::spinOnce();  // Handle callbacks (like litterCallback)
        loop_rate.sleep(); // Sleep to maintain the loop rate
    }

    ros::spin();

    return 0;
}
