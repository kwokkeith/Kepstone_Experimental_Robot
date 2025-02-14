#ifndef LITTER_PLOTTER_H
#define LITTER_PLOTTER_H

#include <ros/ros.h>
#include <ros/param.h>
#include <visualization_msgs/MarkerArray.h>
#include <bumperbot_detection/LitterList.h>

class LitterPlotter
{
public:
    LitterPlotter();

    // Callback function to handle received litter points and convert them to markers
    void litterCallback(const bumperbot_detection::LitterList::ConstPtr& msg);

    // Make this public so it can be accessed in the main function
    ros::Publisher marker_pub_;
    visualization_msgs::MarkerArray marker_array_;

private:
    ros::NodeHandle nh_;  
    ros::Subscriber litter_sub_;               // Subscriber to the LitterList topic
    
    // Variables to store marker configuration
    std::string marker_namespace_;
    geometry_msgs::Vector3 marker_scale_;
    std_msgs::ColorRGBA marker_color_;

    std::set<int> previous_litter_ids_;       // Store previously rendered IDs
};

#endif  
