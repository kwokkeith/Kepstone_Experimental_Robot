#include "bumperbot_detection/litter_plotter.h"

LitterPlotter::LitterPlotter()
{
    ros::NodeHandle pnh_("~");

    // Load marker configuration from parameters
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

    // Load Global parameters for service/topics
    std::string litter_memory_topic_sub_;
    nh_.getParam("/litter_memory/topics/litter_memory", litter_memory_topic_sub_);
    std::string litter_markers_topic_pub_;
    nh_.getParam("/litter_plotter/topics/litter_markers", litter_markers_topic_pub_);

    // Subscribe to the litter memory topic
    litter_sub_ = nh_.subscribe(litter_memory_topic_sub_, 10, &LitterPlotter::litterCallback, this);
    
    // Publisher for marker visualization
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(litter_markers_topic_pub_, 10);
}

void LitterPlotter::litterCallback(const bumperbot_detection::LitterList::ConstPtr& msg)
{
    std::set<int> current_litter_ids;
    visualization_msgs::MarkerArray marker_array;
    
    // Collect IDs of current litter points from the received message
    for (const auto& litter_point : msg->litter_points)
    {
        current_litter_ids.insert(litter_point.id);

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
        marker.lifetime = ros::Duration(); // Persistent markers

        // Set the scale and color from parameters
        marker.scale.x = marker_scale_.x;
        marker.scale.y = marker_scale_.y;
        marker.scale.z = marker_scale_.z;
        marker.color.r = marker_color_.r;
        marker.color.g = marker_color_.g;
        marker.color.b = marker_color_.b;
        marker.color.a = marker_color_.a;

        marker_array.markers.push_back(marker);  // Add to the marker array
    }

    // Identify markers to delete (those in previous set but not in current)
    for (int prev_id : previous_litter_ids_)
    {
        if (current_litter_ids.find(prev_id) == current_litter_ids.end())
        {
            // Create a DELETE marker for this ID
            visualization_msgs::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = ros::Time::now();
            delete_marker.ns = marker_namespace_;
            delete_marker.id = prev_id;
            delete_marker.type = visualization_msgs::Marker::SPHERE;
            delete_marker.action = visualization_msgs::Marker::DELETE;
            
            marker_array.markers.push_back(delete_marker);
        }
    }

    // Update the set of previous litter IDs
    previous_litter_ids_ = current_litter_ids;

    // Publish the updated marker array with additions and deletions
    marker_pub_.publish(marker_array);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_litter_plotter");

    LitterPlotter plotter;

    ros::spin();

    return 0;
}
