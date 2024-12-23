#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sqlite3.h>
#include <sstream>
#include <ros/package.h>

std::string db_path;

sqlite3 *db;
bool polygonStored = false;
bool pathStored = false;
bool mapNameStored = false;

void canvasMessengerCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!polygonStored){
        std::stringstream ss;
        ss << "INSERT INTO messages (topic, content) VALUES ('polygon', '" << msg->data << "');";
        char* errMsg = 0;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Polygon inserted"); }
        polygonStored = true;
    } else if (!pathStored) {
        // Insert path data
        std::stringstream ss;
        ss << "INSERT INTO messages (topic, content) VALUES ('path', '" << msg->data << "');";
        char* errMsg = 0;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Path inserted"); }
        pathStored = true;
    }
}

void mapNameCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!mapNameStored){
        std::stringstream ss;
        ss << "INSERT INTO messages (topic, content) VALUES ('map_name', '" << msg->data << "');";
        char *errMsg = 0;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Map name inserted"); }
        mapNameStored = true;
    }
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
                                   "topic TEXT NOT NULL," \
                                   "content TEXT NOT NULL);";
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