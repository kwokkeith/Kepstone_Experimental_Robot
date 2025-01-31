#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sqlite3.h>
#include <sstream>
#include <ros/package.h>
#include <random>
#include <string>

std::string db_path;

sqlite3 *db;
bool polygonBoundingCoordinatesStored = false;
bool cleaningPathCoordinatesStored = false;
bool mapNameStored = false;
bool bcdPolygonContourCoordinatesStored = false;
bool startPointsStored = false;
bool roiPointsStored = false;

std::string map_name;
std::string start_point;
std::string roi_points;
std::string polygonBounding_coordinates;
std::string cleaning_path_coordinates;
std::string bcdPolygonContour_coordinates;
std::string get_uuid();
void insertPolygonData(const std::string& config_id);
void insertData();


std::string get_uuid() {
    static std::random_device dev;
    static std::mt19937 rng(dev());

    std::uniform_int_distribution<int> dist(0, 15);
    std::uniform_int_distribution<int> dist2(8, 11);

    const char *v = "0123456789abcdef";
    std::string res;
    res.reserve(36);

    for (int i = 0; i < 8; i++) {
        res += v[dist(rng)];
    }
    res += "-";
    for (int i = 0; i < 4; i++) {
        res += v[dist(rng)];
    }
    res += "-4"; // UUID version 4
    for (int i = 0; i < 3; i++) {
        res += v[dist(rng)];
    }
    res += "-";
    res += v[dist2(rng)]; // UUID variant
    for (int i = 0; i < 3; i++) {
        res += v[dist(rng)];
    }
    res += "-";
    for (int i = 0; i < 12; i++) {
        res += v[dist(rng)];
    }

    return res;
}

//TODO: Confirm with keith and Gizelle if the name cleaning_path_coordinates should be changed to waypoints?
//TODO: Add images into db
void insertData(){
    if (mapNameStored && polygonBoundingCoordinatesStored && bcdPolygonContourCoordinatesStored && cleaningPathCoordinatesStored && startPointsStored && roiPointsStored){
        std::string config_id = get_uuid();
        std::stringstream ss;
        ss << "INSERT INTO Config_Table (config_id, map_name, start_point, roi_points, polygonBounding_coordinates, cleaning_path_coordinates) VALUES ('"
              << config_id << "', '" << map_name << "', '" << start_point << "', '" << roi_points << "', '" << polygonBounding_coordinates << "', '" << cleaning_path_coordinates << "');";
            //   << map_name << "', '" << start_point << "', '" << roi_points << "', '" << polygonBounding_coordinates << "', '" << bcdPolygonContour_coordinates << "', '" << cleaning_path_coordinates << "');";
        char* errMsg = nullptr;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Data inserted successfully into Config_Table"); }
        insertPolygonData(config_id);
        // restart when inserted
        startPointsStored = false;
        roiPointsStored = false;
        mapNameStored = false;
        polygonBoundingCoordinatesStored = false;
        cleaningPathCoordinatesStored = false;
        bcdPolygonContourCoordinatesStored = false;

        map_name.clear();
        start_point.clear();
        roi_points.clear();
        polygonBounding_coordinates.clear();
        bcdPolygonContour_coordinates.clear();
        cleaning_path_coordinates.clear();
    }
}

void insertPolygonData(const std::string& config_id){
    if (bcdPolygonContourCoordinatesStored){
        std::stringstream ss;
        ss << "INSERT INTO Polygon_Table (bcdPolygonContour_coordinates, config_id) VALUES ('" << bcdPolygonContour_coordinates << "', '" << config_id << "');";
        char* errMsg = nullptr;
        int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
        if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
        else { ROS_INFO("Data inserted successfully into Polygon_Table"); }
    }
}

void startPointsCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!startPointsStored){
        start_point = msg->data;
        startPointsStored = true;
    }
    insertData();
}

void roiPointsCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!roiPointsStored){
        roi_points = msg->data;
        roiPointsStored = true;
    }
    insertData();
}


void canvasMessengerCallback(const std_msgs::String::ConstPtr& msg) {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    if (!polygonBoundingCoordinatesStored){
        polygonBounding_coordinates = msg->data;
        polygonBoundingCoordinatesStored = true;
    } else if (!bcdPolygonContourCoordinatesStored && polygonBoundingCoordinatesStored && msg->data != polygonBounding_coordinates) {
        bcdPolygonContour_coordinates = msg->data;
        bcdPolygonContourCoordinatesStored = true;
    }else if (!cleaningPathCoordinatesStored && bcdPolygonContourCoordinatesStored && msg->data != bcdPolygonContour_coordinates) {
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
    const char *sql_create_table = "CREATE TABLE IF NOT EXISTS Config_Table (" \
                                   "config_id TEXT PRIMARY KEY," \
                                   "map_name TEXT NOT NULL," \
                                   "start_point TEXT NOT NULL," \
                                   "roi_points TEXT NOT NULL," \
                                   "polygonBounding_coordinates TEXT NOT NULL," \
                                   "cleaning_path_coordinates TEXT NOT NULL);";
    char *errMsg = 0;
    if (sqlite3_exec(db, sql_create_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("Config table created successfully");
    }

    const char *sql_create_polygon_table = "CREATE TABLE IF NOT EXISTS Polygon_Table (" \
                                           "polygon_id INTEGER PRIMARY KEY AUTOINCREMENT," \
                                           "bcdPolygonContour_coordinates TEXT NOT NULL," \
                                           "config_id TEXT," \
                                           "FOREIGN KEY(config_id) REFERENCES Config_Table(config_id));";
    if (sqlite3_exec(db, sql_create_polygon_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("Polygon table created successfully");
    }
    ros::Subscriber start_points_sub = nh.subscribe("/start_point", 1, startPointsCallback);
    ros::Subscriber roi_points_sub = nh.subscribe("/roi_points", 4, roiPointsCallback);
    ros::Subscriber point_sub = nh.subscribe("/canvas_messenger", 1000, canvasMessengerCallback);
    ros::Subscriber map_name_sub = nh.subscribe("/new_map_name", 1000, mapNameCallback);
    ros::spin();
    

    // Close the SQLite database
    sqlite3_close(db);

    return 0;
}