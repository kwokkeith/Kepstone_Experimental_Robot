#ifndef USI_H
#define USI_H

#include <websocket_comm/websocket.h>
#include <std_msgs/String.h>
#include <thread>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
// #include "arm_task_ctrl/UgvMissionState.h"
#include "std_msgs/Bool.h"

class Usi
{
public:
  Usi(ros::NodeHandle &nh);
  ~Usi();
  void post_received_msg_to_handler(const std::string &msg);
  void identify_me();
  void test_robot();

private:
  void handle_ros_event(const boost::system::error_code &error);
  void ros_msg_callback(const std_msgs::String & msg);
  void stop_asio();
  void handle_received_msg(std::string msg);
  bool read_json_from_file(const std::string &filename, nlohmann::json &jsonData);
  // void start_mission(const std::string &filename, int mission_id=-1000);
  // void task_ctrl_mission_callback(const arm_task_ctrl::UgvMissionState &msg);
  // void reset_ugv_safety_io_callback(const std_msgs::Bool &msg);

private:
  ros::NodeHandle nh_;
  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  boost::asio::deadline_timer timer_;
  int timer_duration_;
  // ros::Subscriber task_ctrl_sub_;
  // ros::Subscriber reset_safety_io_sub_;
  ros::Subscriber ros_msg_sub_;
  // ros::Publisher mission_finished_pub_;
  WebSocket* socket_;
};

#endif // USI_H
