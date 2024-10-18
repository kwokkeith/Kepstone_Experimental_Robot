#include "bumperbot_detection/litter_memory.h"
#include "bumperbot_detection/LitterPoint.h"

// Constructor to initialize the LitterMemory node
LitterMemory::LitterMemory() : current_id_(0)
{
    // Set distance threshold for filtering duplicates
    nh_.param<double>("distance_threshold", distance_threshold_, 20.0);  // Load the distance threshold parameter

    // Initialize the subscriber to get litter coordinates in base frame
    litter_sub_ = nh_.subscribe("base_frame/detected_object_coordinates", 10, &LitterMemory::litterCallback, this);

    // Initialize a publisher to publish all remembered litter points
    litter_pub_ = nh_.advertise<bumperbot_detection::LitterList>("litter_memory", 10);

    // Initialize the service to delete litter by ID
    delete_litter_service_ = nh_.advertiseService("litter_memory/delete_litter", &LitterMemory::deleteLitterCallback, this);
}

// Helper function to calculate Euclidean distance between two points
double LitterMemory::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Helper function to check if the new litter point is a duplicate
bool LitterMemory::isDuplicate(const geometry_msgs::PointStamped& new_litter)
{
    // Loop through the remembered litter points
    for (const auto& remembered_litter : remembered_litter_)
    {
        // Calculate the distance between the new litter point and each remembered point
        double distance = calculateDistance(new_litter.point, remembered_litter.point);

        // If the distance is less than the threshold, consider it a duplicate
        if (distance < distance_threshold_)
        {
            return true;  // Duplicate detected
        }
    }

    return false;  // No duplicates found
}

// Helper function to assign a new ID (either recycled or new)
int LitterMemory::getNewID()
{
    if (!available_ids_.empty())
    {
        int recycled_id = available_ids_.front();
        available_ids_.pop();  // Remove from the queue
        return recycled_id;
    }
    else
    {
        return current_id_++;  // If no recycled IDs, increment the current ID
    }
}

// Callback to handle detected litter points
void LitterMemory::litterCallback(const geometry_msgs::PointStamped::ConstPtr& litter_point)
{
    // Check if the new litter point is within the threshold distance from any existing points
    if (!isDuplicate(*litter_point))
    {
        // Create a new LitterPoint with an available or new ID
        bumperbot_detection::LitterPoint new_litter;
        new_litter.id = getNewID();
        new_litter.point = litter_point->point;    // Copy point data
        new_litter.header = litter_point->header;  // Copy header data

        // Store the new litter point in memory
        remembered_litter_.push_back(new_litter);
        ROS_INFO("New litter point remembered with ID: %d", new_litter.id);

        // Publish the entire list of remembered litter points
        publishRememberedLitter();
    }
    else
    {
        ROS_INFO("Litter point ignored (duplicate)");
    }
}

// Publish all remembered litter points
void LitterMemory::publishRememberedLitter()
{
    // Create a LitterList message
    bumperbot_detection::LitterList litter_list_msg;

    for (const auto& litter : remembered_litter_)
    {
        litter_list_msg.litter_points.push_back(litter);  // Push the entire LitterPoint message
    }

    litter_pub_.publish(litter_list_msg);
}

// Service callback to delete litter by ID (If deleted the ID gets added into available pool to reuse)
bool LitterMemory::deleteLitterCallback(bumperbot_detection::DeleteLitter::Request& req, bumperbot_detection::DeleteLitter::Response& res)
{
    // Search for the litter with the given ID
    auto it = std::find_if(remembered_litter_.begin(), remembered_litter_.end(),
                           [&req](const bumperbot_detection::LitterPoint& litter) { return litter.id == req.id; });

    if (it != remembered_litter_.end())
    {
        // Recycle the ID of the deleted litter point
        available_ids_.push(it->id);

        // Remove the litter from memory
        remembered_litter_.erase(it);
        ROS_INFO("Litter point with ID %d deleted", req.id);

        // Set response
        res.success = true;
        res.message = "Litter point deleted successfully";

        // Publish updated litter list
        publishRememberedLitter();
    }
    else
    {
        ROS_WARN("Litter point with ID %d not found", req.id);
        res.success = false;
        res.message = "Litter point not found";
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "litter_memory_node");
    LitterMemory litter_memory;
    ros::spin();

    return 0;
}
