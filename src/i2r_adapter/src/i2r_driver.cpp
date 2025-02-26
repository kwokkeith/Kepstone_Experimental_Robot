#include "i2r_adapter/i2r_driver.hpp"

namespace i2r_driver {
    std::string send_i2r_line_following_mission(ros::NodeHandle& node, std::string& task_id,
        const std::vector<geometry_msgs::PoseStamped>& path)
    {
        task_id = 0; // Temporary task id used because not using RMF

        int _task_id = std::move(std::stoi(task_id)); 
        // std::vector <geometry_msgs::PoseStamped> i2r_waypoint;
        ROS_INFO("---------------> Task id (line following): [%i]", task_id);
        
        // for (const auto& location : path)
        // {
        //     geometry_msgs::PoseStamped _i2r_waypoint;
        //     i2r_waypoint.emplace_back(_i2r_waypoint);
        // }
    
        // Generates the mission with i2r_waypoint
        return mrccc_utils::mission_gen::line_following(_task_id, path);
    }
    
    std::string send_i2r_docking_mission(ros::NodeHandle& node, std::string task_id)
    {
        ROS_INFO("---------------> Task id (docking): [%i]", task_id);
        return std::string("Some docking string for i2r");
    }
    
    
    tf2::Quaternion get_quat_from_yaw(double _yaw)
    {
      tf2::Quaternion quat_tf;
      quat_tf.setRPY(0.0, 0.0, _yaw);
      quat_tf.normalize();
    
      return quat_tf;
    }
    
    } //namespace i2r driver