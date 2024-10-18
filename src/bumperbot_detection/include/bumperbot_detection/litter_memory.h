#ifndef LITTER_MEMORY_H
#define LITTER_MEMORY_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <bumperbot_detection/LitterList.h>
#include <bumperbot_detection/DeleteLitter.h> 
#include <bumperbot_detection/LitterPoint.h>
#include <vector>
#include <queue>    // Recycle ID

class LitterMemory
{
public:
    LitterMemory();
    ~LitterMemory() {}

    // Callback function to handle detected litter points
    void litterCallback(const geometry_msgs::PointStamped::ConstPtr& litter_point);

    // Optional: Publish all remembered litter points
    void publishRememberedLitter();

    // Service callback to delete litter by ID
    bool deleteLitterCallback(bumperbot_detection::DeleteLitter::Request& req, bumperbot_detection::DeleteLitter::Response& res);


private:
    ros::NodeHandle nh_;          
    ros::Subscriber litter_sub_;  // Subscriber to litter points
    ros::Publisher litter_pub_;   // Publisher for remembered litter points
    ros::ServiceServer delete_litter_service_;  // Service to delete litter

    // Vector to store all remembered litter points
    std::vector<bumperbot_detection::LitterPoint> remembered_litter_;

    // Distance threshold to filter out duplicate or close litter points (Distance Filtering)
    double distance_threshold_;

    // Current ID counter for litter points
    int current_id_;

    // Queue to store recycled (available) IDs
    std::queue<int> available_ids_;

    // Helper function to calculate Euclidean distance
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);

    // Helper function to check if a litter point is a duplicate (Using Distance threshold filtering)
    bool isDuplicate(const geometry_msgs::PointStamped& new_litter);

    // Helper function to assign a new ID (recycled or new)
    int getNewID();
};

#endif