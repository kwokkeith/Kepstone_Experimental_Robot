#include "i2r_adapter/mission_gen.hpp"

namespace mrccc_utils{
namespace mission_gen {

    const int TABLE_ID[TABLE_LEN] = {-2,-3,1,2,3};
    const std::string TABLE_NAME[TABLE_LEN] = {"Code Blue", "Code Red", "Resume", "Follow", "Continue"};

    Json::Value GoalData() {
        // Goal points are not used for Ward45
        // Values here are set to default 
        Json::Value data;
        data["description"] = "";
        data["nav_angle_w"] = 1;
        data["nav_angle_x"] = 0;
        data["nav_angle_y"] = 0;
        data["nav_angle_z"] = 0;
        data["nav_map_name"] = "";
        data["nav_pose_x"] = 0;
        data["nav_pose_y"] = 0;
        data["nav_pose_z"] = 0;
        return data;
    }
    
    Json::Value Goal() {
        // Goal points are not used for Ward45
        // Values here are set to default
        Json::Value goal;
        goal["goal_data"] = GoalData();
        goal["goal_id"] = -1; // Not applicable for our usage
        return goal;
    }
    
    Json::Value Pose(float pose_x, float pose_y, tf2::Quaternion quat) {
        Json::Value pose;
        // Getting Quaternion values 
        // tf2::Quaternion quat = i2r_driver::get_quat_from_yaw(yaw);
        pose["description"] = ""; // Point name not used in our case
        pose["nav_angle_w"] = quat.getW();
        pose["nav_angle_x"] = 0.0;
        pose["nav_angle_y"] = 0.0;
        pose["nav_angle_z"] = quat.getZ();
        pose["nav_map_name"] = "";
        pose["nav_pose_x"] = pose_x;
        pose["nav_pose_y"] = pose_y;
        pose["nav_pose_z"] = 0;
        return pose;
    }
    
    Json::Value PointData(int index, float raw_x, float raw_y, tf2::Quaternion quat) {
        // JSON object with attributes required for waypoint data
        Json::Value data;
        data["angle_accuracy"] = 0.3;
        data["drive_forward"] = 1;
        data["group_id"] = 0; // Default >> 0, group_id can be different
        data["horn_mode"] = -1; // Not in use right now
        data["id"] = index; // Point Index
        data["map_x"] = 0.0; // GUI Value, can be ignored
        data["map_y"] = 0.0; // GUI Value, can be ignored
        data["nav_max_turn"] = 0.5; // Consider getting this param from config [preset]
        data["nav_max_vel"] = 1; // Consider getting this param from config [preset]
        data["nav_mode"] = 2; // Default >> 2 [Agile]; 1 [Pure Line Following]
        data["patience"] = 1; // Default is 1
        data["pose"] = Pose(raw_x, raw_y, quat);
        data["position_accuracy"] = 0.3; // Current Default >> 0.3
        data["safety_mode"] = 1;  // Current mode is set to 1 : Full safety
        data["steering_mode"] = "Differential"; // Modes avail: Differential, Ackermann, Omni
        data["time_limit_ms"] = -1; // By default time limit is deactivated.
        data["using_custom_orientation"] = false; // Can be set to true, potentially is for end bed location
        return data;
    }
    
    Json::Value Next(int ID, std::string condition) {
        Json::Value next;
        next["and_condition"] = true;
        next["condition"] = condition;
        next["id"] = ID + 1;
        return next;
    }
    
    Json::Value param_default() {
        Json::Value param;
        Json::Value arr(Json::arrayValue);
        param["retry_count"] = 0;               // Default 0, value not in use
        param["retry_seconds"] = 0;             // Default 0, value not in use
        param["rosparam_write_count_max"] = -1; // Default -1, value not in use
        param["tablettap_but_ids"] = arr;       // Setting up variable as array
        param["tablettap_but_names"] = arr;     // Setting up variable as array
        for (int i = 0; i < TABLE_LEN; i++) {   // Configuring following variable values
            param["tablettap_but_ids"].append(TABLE_ID[i]);
            param["tablettap_but_names"].append(TABLE_NAME[i]);
        };
        param["tablettap_msg_timeout"] = 3;     // Default 3, value not in use
        param["tablettap_print_tts"] = false;   // Default false, value not in use
        param["tablettap_tts"] = "";            // Default  NULL, value not in use
        param["timeout_condition"] =  "";       // Default  NULL, value not in use
        param["timeout_time"] = 0;              // Default  0, value not in use
        return param;
    }
    
    Json::Value param_docking() {
        Json::Value dock;
        // Since inputs from retry onwards are the same, dock is 
        // initiated with param_default first before adding the new variables
        // Docking variables can be changed
        dock = param_default(); 
        dock["align_behaviour/m_LNd"] = 0;
        dock["align_behaviour/m_accuracy_tolerance"] = 0.1;
        dock["align_behaviour/m_align3_tolerance"] = 0.1;
        dock["align_behaviour/m_backward_timer"] = 80;
        dock["align_behaviour/m_compensate_y_constant"] = 0;
        dock["align_behaviour/m_docking_stopping_range"] = 0.5;
        dock["align_behaviour/m_forward_timer"] = 100;
        dock["align_behaviour/m_last_approach_dostate"] = 8;
        dock["align_behaviour/m_last_forward"] = 0.03;
        dock["align_behaviour/m_length_max"] = 0.55;
        dock["align_behaviour/m_length_min"] = 0.45;
        dock["align_behaviour/m_line_type"] = 0;
        dock["align_behaviour/m_retry_allowed"] =2;
        dock["align_behaviour/m_search_area_max_init"] = 3;
        dock["align_behaviour/m_undock_period"] = 60;
        dock["align_behaviour/m_xFar"] = 0.8;
        dock["align_behaviour/m_xNear"] = 0.4;
        dock["align_behaviour/m_yFar"] = 0;
        dock["align_behaviour/m_yNear"] = 0;
        return dock;
    }
    
    std::string Mission_Data (const std::vector<geometry_msgs::PoseStamped> &waypoint) {
        // Processes a list of waypoints and converts into JSON format for robot to follow
        // Processes a set of waypoints and generates a list of JSON objects, each 
        // representing a movement command.
        Json::Value root;
        Json::Value arr(Json::arrayValue);
        root["all_goals"] = arr;
        root["exit_id"] = -1; // [-1] Task finishes after last id in the line is reached
        root["goal"] = Goal();
        root["line_name"] = "generic_line";
        root["line_type"] = 0;
        root["repeat_count"] = 0;
        root["start_id"] = 0; // Determines the ID of the starting Node
        root["points"] = arr;
        int index = 0;
        for (auto& wp : waypoint) {
            // Each waypoint, x, y and yaw
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(wp.pose.orientation, tf2_quat); // Convert geometry_msgs::Quaternion to tf2::Quarternion
            root["points"].append(PointData(index, wp.pose.position.x, wp.pose.position.y, tf2_quat)); 
            index++;
        };
        Json::FastWriter print;
        std::string s = print.write(root);
        return s;
    } 
    
    Json::Value Mission(int mission_type, const std::vector<geometry_msgs::PoseStamped>& waypoint) {    
        // Creates JSON data for different types of missions (line following, docking, etc.)
        Json::Value root;
        Json::Value arr(Json::arrayValue);
        root["description"] = "generic_line";
        root["gui"]["x"] = 800;
        root["gui"]["y"] = 600;
        root["next"] = arr;
        root["submission_uid"] = 1; // Default starting value is 1 
        root["type"] = mission_type;
        root["rosparams"] = param_default();
        if (mission_type == 10001) { // Line following code (10001)
            // Line Following
            root["name"] = "line_following";
            root["next"].append(Next(1, "line_following_status == 2"));
            root["rosparams"]["mission_data"] = Mission_Data(waypoint);
        } else {
            // Unknown Mode
            root["name"] = "Unknown";
        }
        return root;
    }
    
    Json::Value Mission_Dock() {
        Json::Value root;
        Json::Value arr(Json::arrayValue);
        root["description"] = "";
        root["gui"]["x"] = 800;
        root["gui"]["y"] = 600;
        root["next"] = arr;
        root["submission_uid"] = 1; // Default starting value is 1 
        root["type"] = 2; // Docking command: 2
        root["rosparams"] = param_default();
        root["name"] = "dock";
        root["next"].append(Next(1, "dock_status == 2"));
        root["rosparams"] = param_docking();
        return root;
    }
    
    Json::Value mission_endl() {
        Json::Value completion;
        Json::Value arr(Json::arrayValue);
        completion["description"] = "generic_line";
        completion["gui"]["x"] = 860;
        completion["gui"]["y"] = 690;   
        completion["name"] = "completed";
        completion["next"] = arr;
        completion["rosparams"] = param_default();
        completion["submission_uid"] = 2;
        completion["type"] = -3;
        return completion;
    }
    
    Json::Value cmd_header(int cmd_id, std::string name, int task_id) {
            Json::Value header;
            header["clientname"] = name;
            header["cmd"] = cmd_id;
            header["dest_clientname"] = "magni";
            header["requestid"] = task_id;
            header["subcmd"] = 254;
            return header;
        }

    // Publish navigation waypoints directly to move_base
    std::string line_following(const int& task_id, 
        const std::vector<geometry_msgs::PoseStamped>& waypoints) 
        // Generate complete JSON messages to control the robot
        // Complete mission request including a command header (cmd_header())
        {
            Json::Value root;
            Json::Value arr(Json::arrayValue);
            Json::StreamWriterBuilder builder;
            const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
            std::ofstream output;
            
            // building root data
            root["header"] = cmd_header(13, "CHART", task_id); // line_following cmd: 13
            root["payload"]["mission_id"] = task_id;    // task_id from RMF
            root["payload"]["mission_repeat_count"] = 0; // Set to 0 always, RMF handles loop
            root["payload"]["mission_version"] = 1;
            root["payload"]["missions"] = arr;
            root["payload"]["missions"].append(Mission(10001, waypoints)); // Line following mission
            root["payload"]["missions"].append(mission_endl());
            root["payload"]["start_id"] = 1; // start_id of submission_uid
            
            std::cout<<"Sending path ending at position "<<
                waypoints.back().pose.position.x <<
                waypoints.back().pose.position.y <<
                waypoints.back().pose.orientation <<std::endl;

            Json::FastWriter print;
            std::string line_following = print.write(root);

            // DEBUGGING: Saving root data into text file
            // output.open("line_following.txt");
            // writer->write(root, &output);
            // output.close();
            
            // Returns the JSON required to be sent to the I2R API
            return line_following;
    }

    std::string abort_all() {
        Json::Value root;
        
        // building root data
        root["header"] = cmd_header(10, "CHART", 29); // line_following cmd: 13
        root["payload"];
        
        Json::FastWriter print;
        std::string abort_all = print.write(root);
        
        return abort_all;
    }
    
    std::string abort(int task_id) {
        // Creates JSON message to abort robot's task
        Json::Value root;
        Json::StreamWriterBuilder builder;
        const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        std::ofstream output;
    
        root["header"] = cmd_header(11, "i2r-adapter", task_id); // cmd: 10 [abort active]; 11 [abort all]
        root["payload"] = {};
        Json::FastWriter print;
        std::string abort_mission = print.write(root);
    
        // DEBUGGING: Saving root data into text file
        // output.open("abort.txt");
        // writer->write(root, &output);
        // output.close();
        
        return abort_mission;
    }
    

    std::string dock(int task_id) {
        // Generates complete JSON messages to control robot to dock
        Json::Value root;
        Json::Value arr(Json::arrayValue);
        Json::StreamWriterBuilder builder;
        const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        std::ofstream output;
    
        root["header"] = cmd_header(2, "CHART", task_id); // dock cmd: 2
        root["payload"]["mission_id"] = task_id;    // task_id from RMF
        root["payload"]["mission_repeat_count"] = 0; // Set to 0 always, RMF handles loop
        root["payload"]["mission_version"] = 1;
        root["payload"]["missions"] = arr;
        root["payload"]["missions"].append(Mission_Dock());
        root["payload"]["missions"].append(mission_endl());
        root["payload"]["start_id"] = 1; // start_id of submission_uid
    
        Json::FastWriter print;
        std::string docking_mission = print.write(root);
    
        // DEBUGGING: Saving root data into text file
        // output.open("dock.txt");
        // writer->write(root, &output);
        // output.close();
        return docking_mission;
    }

    std::string identifyMe() {
        // Get robot to identify itself
        Json::Value root;
        Json::StreamWriterBuilder builder;
        const std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        std::ofstream output;
        root["header"]["clientlocation"] = "full2dmap01";
        root["header"]["clientname"] = "cag_vacuum_amr";
        root["header"]["clienttype"] = 1;
        root["header"]["cmd"] = 6;
        root["header"]["type"] = 1;
        Json::FastWriter print;
        std::string identity = print.write(root);
        // DEBUGGING: Saving root data into text file
        // output.open("dock.txt");
        // writer->write(root, &output);
        // output.close();
        return identity;
    }

} // namespace mission_gen
} // namespace mrccc_utils
