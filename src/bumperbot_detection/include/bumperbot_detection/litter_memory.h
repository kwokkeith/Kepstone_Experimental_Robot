#ifndef LITTER_MEMORY_H
#define LITTER_MEMORY_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <bumperbot_detection/LitterList.h>
#include <bumperbot_detection/DeleteLitter.h>
#include <bumperbot_detection/AddLitter.h>
#include <bumperbot_detection/GetLitterList.h> 
#include <bumperbot_detection/LitterPoint.h>
#include <bumperbot_detection/DetectedLitterPoint.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <queue>    // Recycle ID
#include <std_srvs/Trigger.h>
#include <nav_msgs/OccupancyGrid.h>

class LitterMemory
{
public:
    LitterMemory();
    ~LitterMemory() {}
    // Callback function to handle detected litter points
    void litterCallback(const geometry_msgs::PointStamped::ConstPtr& litter_point);
    // Publish all remembered litter points
    void publishRememberedLitter();
    // Service callback to get memory of litter
    bool getLitterListCallback(bumperbot_detection::GetLitterList::Request& req,
                                bumperbot_detection::GetLitterList::Response& res);
    // Service callback to add litter to memory
    bool addLitterCallback(bumperbot_detection::AddLitter::Request& req,
                                     bumperbot_detection::AddLitter::Response& res);
    // Service callback to delete litter by ID
    bool deleteLitterCallback(bumperbot_detection::DeleteLitter::Request& req, bumperbot_detection::DeleteLitter::Response& res);
    // Service callback to clear litter memory
    bool clearMemoryCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    // Callback function to handle costmap data
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap);

private:
    ros::NodeHandle nh_;          
    ros::NodeHandle pnh_;
    ros::Subscriber litter_sub_;                // Subscriber to litter points
    ros::Subscriber global_costmap_sub_;        // Subscriber to the Global Costmap Topic
    ros::Publisher litter_pub_;                 // Publisher for remembered litter points
    ros::Publisher new_litter_pub_;             // Publisher for new litter
    ros::Publisher detected_litter_raw_pub_; // Publisher for detected litter on map coordinates without removing duplicates
    ros::ServiceServer delete_litter_service_;  // Service to delete litter
    ros::ServiceServer get_litter_service_;     // Service to get litter from memory
    ros::ServiceServer add_litter_service_;     // Service to add litter to memory
    ros::ServiceServer clear_memory_service_;   // Service to clear the litter memory

    // TF2 Buffer and Listener to manage transforms
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;  // Initialize with the buffer in constructor

    // Vector to store all remembered litter points
    std::vector<bumperbot_detection::LitterPoint> remembered_litter_;

    // To store costmap data
    nav_msgs::OccupancyGrid costmap_;

    // Distance threshold to filter out duplicate or close litter points (Distance Filtering)
    double distance_threshold_;

    // Current ID counter for litter points
    int current_id_;

    // Queue to store recycled (available) IDs
    std::queue<int> available_ids_;

    // Helper function to calculate Euclidean distance
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

    // Helper function to check if a litter point is a duplicate (Using Distance threshold filtering)
    bumperbot_detection::LitterPoint isDuplicate(const bumperbot_detection::LitterPoint& new_litter);

    // Helper function to assign a new ID (recycled or new)
    int getNewID();

    // Helper function to check if a point is too near to an obstacle
    bool isTooNearObstacle(const geometry_msgs::Point& point);
};

#endif