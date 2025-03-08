#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sqlite3.h>
#include <sstream>
#include <ros/package.h>
#include <random>
#include <string>
#include <std_msgs/Bool.h>

std::string db_path;

sqlite3 *db;
bool polygonBoundingCoordinatesStored = false;
bool cleaningPathCoordinatesStored = false;
bool mapNameStored = false;
bool bcdPolygonContourCoordinatesStored = false;
bool startPointsStored = false;
bool roiPointsStored = false;
bool navigation_waypointsStored = false;

std::string map_name;
std::string start_point;
std::string roi_points;
std::string polygonBounding_coordinates;
std::string cleaning_path_coordinates;
std::string bcdPolygonContour_coordinates;
std::string navigation_waypoints;
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

void insertData(){
    if (mapNameStored && polygonBoundingCoordinatesStored && bcdPolygonContourCoordinatesStored && cleaningPathCoordinatesStored && startPointsStored && roiPointsStored && navigation_waypointsStored){
        std::string config_id = get_uuid();
        std::stringstream ss;
        ss << "INSERT INTO Config_Table (config_id, map_name, start_point, roi_points, polygonBounding_coordinates, cleaning_path_coordinates, navigation_waypoints) VALUES ('"
              << config_id << "', '" << map_name << "', '" << start_point << "', '" << roi_points << "', '" << polygonBounding_coordinates << "', '" << cleaning_path_coordinates << "', '" << navigation_waypoints << "');";
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
        navigation_waypointsStored = false;

        map_name.clear();
        start_point.clear();
        roi_points.clear();
        polygonBounding_coordinates.clear();
        bcdPolygonContour_coordinates.clear();
        cleaning_path_coordinates.clear();
        navigation_waypoints.clear();
    }
}

// void insertPolygonData(const std::string& config_id){
//     if (bcdPolygonContourCoordinatesStored){
//         std::stringstream ss;
//         ss << "INSERT INTO Polygon_Table (bcdPolygonContour_coordinates, config_id) VALUES ('" << bcdPolygonContour_coordinates << "', '" << config_id << "');";
//         char* errMsg = nullptr;
//         int rc = sqlite3_exec(db, ss.str().c_str(), 0, 0, &errMsg);
//         if (rc != SQLITE_OK) { ROS_ERROR("SQL error: %s", errMsg); sqlite3_free(errMsg); }
//         else { ROS_INFO("Data inserted successfully into Polygon_Table"); }
//     }
// }

void insertPolygonData(const std::string& config_id) {
    if (bcdPolygonContourCoordinatesStored) {
        std::stringstream ss(bcdPolygonContour_coordinates);
        std::string item;
        int polygon_index = 0;
        while (std::getline(ss, item, ']')) {
            size_t start = item.find('[');
            if (start != std::string::npos) {
                std::string coordinates = item.substr(start + 1);

                // Escape single quotes in coordinates
                size_t pos = 0;
                while ((pos = coordinates.find("'", pos)) != std::string::npos) {
                    coordinates.insert(pos, "'");
                    pos += 2;
                }

                std::stringstream insert_ss;
                insert_ss << "INSERT INTO Polygon_Table (bcdPolygonContour_coordinates, config_id, polygon_index) VALUES ('" << coordinates << "', '" << config_id << "', " << polygon_index << ");";

                // Print the SQL statement for debugging
                ROS_INFO("Executing SQL: %s", insert_ss.str().c_str());

                char* errMsg = nullptr;
                int rc = sqlite3_exec(db, insert_ss.str().c_str(), 0, 0, &errMsg);
                if (rc != SQLITE_OK) {
                    ROS_ERROR("SQL error: %s", errMsg);
                    sqlite3_free(errMsg);
                } else {
                    ROS_INFO("Data inserted successfully into Polygon_Table");
                }

                polygon_index++;
            }
        }
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
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
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
    // ROS_INFO("Boolean States => polygonBoundingCoordinatesStored: %s, bcdPolygonContourCoordinatesStored: %s, cleaningPathCoordinatesStored: %s",
    //           polygonBoundingCoordinatesStored ? "true" : "false",
    //           bcdPolygonContourCoordinatesStored ? "true" : "false",
    //           cleaningPathCoordinatesStored ? "true" : "false");
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

void navigation_waypointsCallback(const std_msgs::String::ConstPtr& msg) {
    if (!navigation_waypointsStored){
        navigation_waypoints = msg->data;
        navigation_waypointsStored = true;
    }
    insertData();
}

void shutdownCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        ros::shutdown();
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
    const char *sql_create_table = "CREATE TABLE IF NOT EXISTS Config_Table (" \
                                   "config_id TEXT PRIMARY KEY," \
                                   "map_name TEXT NOT NULL," \
                                   "start_point TEXT NOT NULL," \
                                   "roi_points TEXT NOT NULL," \
                                   "polygonBounding_coordinates TEXT NOT NULL," \
                                   "cleaning_path_coordinates TEXT NOT NULL," \
                                   "navigation_waypoints TEXT NOT NULL);";
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
                                           "polygon_index INTEGER," \
                                           "FOREIGN KEY(config_id) REFERENCES Config_Table(config_id));";
    if (sqlite3_exec(db, sql_create_polygon_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("Polygon table created successfully");
    }

    const char *sql_create_jobs_table = "CREATE TABLE IF NOT EXISTS Jobs_Table (" \
                                        "job_id TEXT," \
                                        "config_id TEXT," \
                                        "sequence_no INTEGER ,"\
                                        "PRIMARY KEY(job_id, sequence_no),"\
                                        "FOREIGN KEY(config_id) REFERENCES Config_Table(config_id));";
                                        
    if (sqlite3_exec(db, sql_create_jobs_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
    return 1;
    } else {
        ROS_INFO("jobs table created successfully");
    }

    const char *sql_create_job_order_table = "CREATE TABLE IF NOT EXISTS Job_Order_Table (" \
                                        "job_id TEXT NOT NULL," \
                                        "config_id TEXT NOT NULL," \
                                        "PRIMARY KEY(job_id, config_id)," \
                                        "FOREIGN KEY(job_id) REFERENCES Jobs_Table(job_id)," \
                                        "FOREIGN KEY(config_id) REFERENCES Config_Table(config_id));";
                                        
    if (sqlite3_exec(db, sql_create_job_order_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
    return 1;
    } else {
        ROS_INFO("jobs_order table created successfully");
    }

    const char *sql_create_schedule_table = "CREATE TABLE IF NOT EXISTS Schedule_Table (" \
                                    "schedule_id TEXT PRIMARY KEY," \
                                    "schedule_name TEXT," \
                                    "start_time TEXT NOT NULL CHECK (start_time GLOB '[0-2][0-9]:[0-5][0-9]:[0-5][0-9]')," \
                                    "start_date TEXT CHECK (start_date GLOB '[0-3][0-9]-[0-1][0-9]-[0-9][0-9][0-9][0-9]')," \
                                    "recurrence TEXT NOT NULL CHECK (recurrence IN ('NoRepeat', 'Weekly', 'Monthly')) DEFAULT 'NoRepeat');";
                                        
    if (sqlite3_exec(db, sql_create_schedule_table, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", errMsg);
        sqlite3_free(errMsg);
    return 1;
    } else {
        ROS_INFO("schedule table created successfully");
    }
    
    const char *sql_create_schedule_job_link = "CREATE TABLE IF NOT EXISTS Schedule_Job_Link (" \
                                           "schedule_id TEXT NOT NULL," \
                                           "sequence_no INTEGER NOT NULL," \
                                           "job_id TEXT NOT NULL," \
                                           "PRIMARY KEY(schedule_id, sequence_no)," \
                                           "FOREIGN KEY(schedule_id) REFERENCES Schedule_Table(schedule_id)," \
                                           "FOREIGN KEY(job_id, sequence_no) REFERENCES Jobs_Table(job_id, sequence_no));";

    if (sqlite3_exec(db, sql_create_schedule_job_link, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error creating schedule-job link table: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("Schedule-Job link table created successfully");
    }
    
    const char *sql_create_job_status = "CREATE TABLE IF NOT EXISTS JobStatus (" \
                                    "job_id TEXT PRIMARY KEY," \
                                    "status TEXT NOT NULL CHECK (status IN ('failed', 'scheduled', 'active', 'completed'))," \
                                    "FOREIGN KEY(job_id) REFERENCES Jobs_Table(job_id));";

    if (sqlite3_exec(db, sql_create_job_status, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error creating JobStatus table: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("JobStatus table created successfully");
    }

    const char *sql_create_contains = "CREATE TABLE IF NOT EXISTS ContainsTable (" \
                                "job_id TEXT," \
                                "sequence_no INTEGER NOT NULL," \
                                "PRIMARY KEY(job_id, sequence_no),"\
                                "FOREIGN KEY(sequence_no) REFERENCES Jobs_Table(sequence_no)," \
                                "FOREIGN KEY(job_id) REFERENCES Jobs_Table(job_id));";

    if (sqlite3_exec(db, sql_create_contains, 0, 0, &errMsg) != SQLITE_OK) {
        ROS_ERROR("SQL error creating JobStatus table: %s", errMsg);
        sqlite3_free(errMsg);
        return 1;
    } else {
        ROS_INFO("JobStatus table created successfully");
    }

    ros::Subscriber start_points_sub = nh.subscribe("/start_point", 1, startPointsCallback);
    ros::Subscriber roi_points_sub = nh.subscribe("/roi_points", 4, roiPointsCallback);
    ros::Subscriber point_sub = nh.subscribe("/canvas_messenger", 1000, canvasMessengerCallback);
    ros::Subscriber map_name_sub = nh.subscribe("/new_map_name", 1000, mapNameCallback);
    ros::Subscriber navigation_waypoints_sub = nh.subscribe("/coverage_planner/navigation_waypoints", 1000, navigation_waypointsCallback);
    
    ros::Subscriber shutdown_sub = nh.subscribe("/shutdown_bool", 1, shutdownCallback);

    // Start an asynchronous spinner to process callbacks concurrently
    ros::AsyncSpinner spinner(0); // Use threads equal to the number of CPU cores
    spinner.start();

    // Wait until ros::shutdown() is called (e.g., via shutdownCallback)
    ros::waitForShutdown();
    

    // Close the SQLite database
    sqlite3_close(db);

    return 0;
}