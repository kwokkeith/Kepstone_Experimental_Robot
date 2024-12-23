#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sqlite3.h>
#include <sstream>
#include <ros/package.h>

std::string db_path;

sqlite3 *db;
bool polygonBoundingCoordinatesStored = false;
bool cleaningPathCoordinatesStored = false;
bool mapNameStored = false;

std::string map_name;
std::string polygonBounding_coordinates;
std::string cleaning_path_coordinates;
//TODO: Confirm with keith and Gizelle if the name cleaning_path_coordinates should be changed to waypoints?
//TODO: Add images into db
void insertData(){
    if (mapNameStored && polygonBoundingCoordinatesStored && cleaningPathCoordinatesStored){
        std::stringstream ss;
        ss << "INSERT INTO messages (map_name, polygonBounding_coordinates, cleaning_path_coordinates) VALUES ('"
              << map_name << "', '" << polygonBounding_coordinates << "', '" << cleaning_path_coordinates << "');";
        char* errMsg = nullptr;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Data inserted successfully"); }

        mapNameStored = false;
        polygonBoundingCoordinatesStored = false;
        cleaningPathCoordinatesStored = false;
        map_name.clear();
        polygonBounding_coordinates.clear();
        cleaning_path_coordinates.clear();
    }
}

void canvasMessengerCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!polygonBoundingCoordinatesStored){
        polygonBounding_coordinates = msg->data;
        polygonBoundingCoordinatesStored = true;
    } else if (!cleaningPathCoordinatesStored && msg->data != polygonBounding_coordinates) {
        cleaning_path_coordinates = msg->data;
        cleaningPathCoordinatesStored = true;
    }
    insertData();
}

void mapNameCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!mapNameStored){
        map_name = msg->data;
        mapNameStored = true;
    }
    insertData();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;

    std::string GUI_package_path = ros::package::getPath("bumperbot_graphical_interface");
    db_path= GUI_package_path + "/db/coverage_zones.db";
    // Open the SQLite database
    if (sqlite3_open(db_path.c_str(), &db)) {
        ROS_ERROR("Can't open database: %s", sqlite3_errmsg(db));
        return 1;
    } else {
        ROS_INFO("Opened database successfully");
    }

    // Create the table if it doesn't exist
    const char *sql_create_table = "CREATE TABLE IF NOT EXISTS messages (" \
                                   "id INTEGER PRIMARY KEY AUTOINCREMENT," \
                                   "map_name TEXT NOT NULL," \
                                   "polygonBounding_coordinates TEXT NOT NULL," \
                                   "cleaning_path_coordinates TEXT NOT NULL);";
    char *errMsg = 0;
    if (sqlite3_exec(db, sql_create_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("Table created successfully");
    }

    ros::Subscriber point_sub = nh.subscribe("/canvas_messenger", 1000, canvasMessengerCallback);
    ros::Subscriber map_name_sub = nh.subscribe("/new_map_name", 1000, mapNameCallback);
    ros::spin();
    

    // Close the SQLite database
    sqlite3_close(db);

    return 0;
}