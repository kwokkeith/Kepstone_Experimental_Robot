#include "bumperbot_detection/litter_memory.h"

// Constructor to initialize the LitterMemory node
LitterMemory::LitterMemory() 
   : current_id_(0),
     pnh_("~"),
     tf_listener_(tf_buffer_)
{
    // Get global configurations
    // Service Servers
    std::string get_litter_list_srv_;
    std::string add_litter_list_srv_;
    std::string delete_litter_srv_;
    std::string clear_memory_srv_;
    nh_.getParam("/litter_memory/services/get_litter_list", get_litter_list_srv_);
    nh_.getParam("/litter_memory/services/add_litter", add_litter_list_srv_);
    nh_.getParam("/litter_memory/services/delete_litter", delete_litter_srv_);
    nh_.getParam("/litter_memory/services/clear_memory", clear_memory_srv_);
    // Service Clients

    // Topic Publishers
    std::string litter_memory_topic_pub_;
    std::string new_litter_topic_pub_;
    std::string detected_litter_raw_topic_pub_;
    nh_.getParam("/litter_memory/topics/litter_memory", litter_memory_topic_pub_);
    nh_.getParam("/litter_memory/topics/new_litter", new_litter_topic_pub_);
    nh_.getParam("/litter_memory/topics/detected_litter_raw", detected_litter_raw_topic_pub_);
    // Topic Subscribers
    std::string detected_object_coordinates_topic_sub_;
    std::string global_costmap_topic_sub_;
    nh_.getParam("/litter_detection/topics/detected_object_coordinates_base", detected_object_coordinates_topic_sub_);
    nh_.getParam("/move_base/topics/global_costmap", global_costmap_topic_sub_);

    // Set distance threshold for filtering duplicates
    pnh_.param<double>("distance_threshold", distance_threshold_, 5.0);  // Load the distance threshold parameter

    // Initialize the subscriber to get litter coordinates in base frame
    litter_sub_ = nh_.subscribe(detected_object_coordinates_topic_sub_, 10, &LitterMemory::litterCallback, this);

    // Initialize the subscriber to get the global costmap
    global_costmap_sub_ = nh_.subscribe(global_costmap_topic_sub_, 10, &LitterMemory::costmapCallback, this);

    // Initialize a publisher to publish all remembered litter points
    litter_pub_ = nh_.advertise<bumperbot_detection::LitterList>(litter_memory_topic_pub_, 10);

    // Initialize a publisher to publish new litter points
    new_litter_pub_ = nh_.advertise<bumperbot_detection::DetectedLitterPoint>(new_litter_topic_pub_, 10);

    // Initialize the publisher to publish detected litter points (without removing duplicates) according to map coordinates
    detected_litter_raw_pub_ = nh_.advertise<geometry_msgs::Point>(detected_litter_raw_topic_pub_, 10);

    // Initialize the service to get the remembered litter points
    get_litter_service_ = nh_.advertiseService(get_litter_list_srv_, &LitterMemory::getLitterListCallback, this);

    // Initialize the service to add a new litter point
    add_litter_service_ = nh_.advertiseService(add_litter_list_srv_, &LitterMemory::addLitterCallback, this);

    // Initialize the service to delete litter by ID
    delete_litter_service_ = nh_.advertiseService(delete_litter_srv_, &LitterMemory::deleteLitterCallback, this);

    // Initialize the service to clear all litter
    clear_memory_service_ = nh_.advertiseService(clear_memory_srv_, &LitterMemory::clearMemoryCallback, this);
}


// Helper function to calculate Euclidean distance between two points
double LitterMemory::calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}


// Helper function to check if the new litter point is a duplicate
bumperbot_detection::LitterPoint LitterMemory::isDuplicate(const bumperbot_detection::LitterPoint& new_litter)
{
    // Loop through the remembered litter points
    for (const auto& remembered_litter : remembered_litter_)
    {
        // Calculate the distance between the new litter point and each remembered point
        double distance = calculateDistance(new_litter.point, remembered_litter.point);

        // If the distance is less than the threshold, consider it a duplicate
        if (distance < distance_threshold_)
        {
            return remembered_litter;  // Duplicate detected, return the duplicate litter in the memory
        }
    }

    return new_litter;  // No duplicates found
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

void LitterMemory::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap)
{
    costmap_ = *costmap;
}

bool LitterMemory::isTooNearObstacle(const geometry_msgs::Point& point)
{
    if (costmap_.data.empty()) {
        ROS_WARN("Error receiving costmap data.");
        return false;
    }

    double resolution = costmap_.info.resolution;
    double origin_x = costmap_.info.origin.position.x;
    double origin_y = costmap_.info.origin.position.y; 
    int width = costmap_.info.width;
    int height = costmap_.info.height;

    // Convert the point coordinates to costmap grid coordinates
    int grid_x = (point.x - origin_x) / resolution;
    int grid_y = (point.y - origin_y) / resolution;

    // Check if the coordinates are within the bounds of the costmap
    if (grid_x < 0 || grid_y < 0 || grid_x >= width || grid_y >= height) {
        ROS_WARN("Point is out of costmap bounds.");
        return false;
    }

    // Get the index of the costmap data array 
    int index = grid_y * width + grid_x;
    int cost = costmap_.data[index];
    ROS_INFO("Cost at point (%f, %f): %d", point.x, point.y, cost);

    // Check if the cost is greater than 0 (near an obstacle)
    return cost > 0;
}

// Callback to handle detected litter points
void LitterMemory::litterCallback(const geometry_msgs::PointStamped::ConstPtr& litter_point)
{   
    // Transform litter point from base to map frame
    geometry_msgs::PointStamped litter_in_map_frame;

    try {
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));

        // Perform the transformation to the map frame
        tf2::doTransform(*litter_point, litter_in_map_frame, transform_stamped);
        ROS_DEBUG("Transformed litter point to map frame: x=%f, y=%f, z=%f", litter_in_map_frame.point.x, litter_in_map_frame.point.y, litter_in_map_frame.point.z);

        // Extract geometry_msgs::Point and publish it as raw detection 
        detected_litter_raw_pub_.publish(litter_in_map_frame.point);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Could not transform litter coordinates from camera_frame to map: %s", ex.what());
        return;  // Skip further processing if transform fails
    }

    // Check if the litter point is too near obstacles, skip if too near
    if (isTooNearObstacle(litter_in_map_frame.point))
    {
        ROS_WARN("Litter point too near obstacles, ignoring.");
        return;  
    }

    // Check if the new litter point is within the threshold distance from any existing points
    bumperbot_detection::DetectedLitterPoint detected_litter;

    // Create a new LitterPoint with an available or new ID
    bumperbot_detection::LitterPoint new_litter;
    new_litter.id = -1;                               // Temporary litter id
    new_litter.point = litter_in_map_frame.point;     // Copy point data
    new_litter.header = litter_in_map_frame.header;   // Copy header data
    
    bumperbot_detection::LitterPoint final_detected_litter = isDuplicate(new_litter);   // Gets the final litter after checking for duplicates
    detected_litter.litter_point = final_detected_litter;

    // final_detected_litter would be same as the new_litter if it is a new litter and not a duplicate
    if (final_detected_litter.point == new_litter.point)
    {
        // Store the new litter point in memory
        detected_litter.litter_point.id = getNewID();         // Get a new id from the queue system
        remembered_litter_.push_back(detected_litter.litter_point);
        ROS_INFO("New litter point remembered with ID: %d", detected_litter.litter_point.id);

        detected_litter.duplicate = false;

        // Publish the entire list of remembered litter points
        publishRememberedLitter();
    }
    else
    {
        detected_litter.duplicate = true;
        ROS_INFO("Litter point ignored (duplicate)");
    }
    // Publish the newly detected litter with the duplicate key for whether it was a deemed a duplicate
    new_litter_pub_.publish(detected_litter);
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


// Service callback to clear the litter memory of any litters (reset the memory)
bool LitterMemory::clearMemoryCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    remembered_litter_.clear();              // Clear the remembered litter list
    current_id_ = 0;                         // Reset the current id to allocate
    available_ids_ = std::queue<int>();      // Reset the available id queue

    publishRememberedLitter();               // Republish the new cleared litter memory 

    // Response of service
    ROS_INFO("Cleared all remembered litter.");
    res.success = true;
    res.message = "Cleared all remembered litter.";
    return true;
}


bool LitterMemory::getLitterListCallback(bumperbot_detection::GetLitterList::Request& req,
                                         bumperbot_detection::GetLitterList::Response& res)
{
    // Fill the response with the remembered litter list
    for (const auto& litter : remembered_litter_)
    {
        res.remembered_litter.litter_points.push_back(litter);  
    }

    return true;  // Return success
}


bool LitterMemory::addLitterCallback(bumperbot_detection::AddLitter::Request& req,
                                     bumperbot_detection::AddLitter::Response& res)
{
    // Check if the new litter point is within the threshold distance from any existing points
    // Create a new LitterPoint with an available or new ID
    bumperbot_detection::LitterPoint new_litter;
    new_litter.id = -1;                            // Temporary litter ID
    new_litter.point = req.litter_point.point;     // Copy point data
    new_litter.header = req.litter_point.header;   // Copy header data

    if (isDuplicate(new_litter).point == new_litter.point)
    {
        new_litter.id = getNewID();                // Get new ID for litter from ID queue

        // Store the new litter point in memory
        remembered_litter_.push_back(new_litter);
        ROS_INFO("New litter point added manually with ID: %d", new_litter.id);

        // Publish the entire list of remembered litter points
        publishRememberedLitter();

        res.success = true;
        res.message = "Litter point added successfully";
    }
    else
    {
        ROS_WARN("Litter point ignored (duplicate)");
        res.success = false;
        res.message = "Litter point is too close to an existing one (duplicate)";
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
