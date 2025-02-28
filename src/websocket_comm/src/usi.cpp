#include "websocket_comm/usi.h"
#include <fstream>

Usi::Usi(ros::NodeHandle &nh):
  nh_(nh),
  work_(boost::asio::make_work_guard(io_context_)),
  timer_(io_context_),
  timer_duration_(1)
{
  ros_msg_sub_ = nh_.subscribe("/websocket_msg_out",1,&Usi::ros_msg_callback,this);
  // task_ctrl_sub_ = nh_.subscribe("/start_ugv_mission",1,&Usi::task_ctrl_mission_callback,this);
  // reset_safety_io_sub_ = nh_.subscribe("/reset_ugv_safety_io",1,&Usi::reset_ugv_safety_io_callback,this);
  // mission_finished_pub_ =  nh_.advertise<arm_task_ctrl::UgvMissionState>("/ugv_mission_finish_state",1);

  // The SSL context is required, and holds certificates
  ssl::context ctx{ssl::context::tlsv12_client};

  // This holds the root certificate used for verification
  std::string certification_file, key_file;
  if(!nh_.getParam("/websocket_comm/certification", certification_file))
  {
    std::cerr<<"Fail to get certification file, exit."<<std::endl;
    exit(-1);
  }

  if(!nh_.getParam("/websocket_comm/key", key_file))
  {
    std::cerr<<"Fail to get key file, exit."<<std::endl;
    exit(-1);
  }
  ctx.use_certificate_file(certification_file, ssl::context::pem);
  ctx.set_password_callback([](std::size_t,
                            boost::asio::ssl::context_base::password_purpose)
  {
    return "pass_default_gui1";
  });
  ctx.use_rsa_private_key_file(key_file, ssl::context::pem);

  socket_ = new WebSocket(ctx, this);

  socket_->connect();

  // Queue the main ros loop
  timer_.expires_from_now(boost::posix_time::milliseconds(timer_duration_));
  timer_.async_wait(boost::bind(&Usi::handle_ros_event, this, boost::asio::placeholders::error));

  // Call identify me automatically
  boost::asio::post(io_context_, boost::bind(&Usi::identify_me,this));

  // Start the asio service event loop in the main thread
  io_context_.run();
}

Usi::~Usi()
{
  socket_->disconnect();
  delete socket_;
}

// void Usi::task_ctrl_mission_callback(const arm_task_ctrl::UgvMissionState &msg)
// {
//   start_mission(msg.mission_filename, msg.mission_id);
// }

// void Usi::reset_ugv_safety_io_callback(const std_msgs::Bool &msg)
// {
//   nlohmann::json jsonObject = {
//     {"header", {
//        {"clientname", "isera2-hawkeye1"},
//        {"cmd", 23},
//        {"dest_clientname", "CAG_1"},
//        {"requestid", 11},
//        {"subcmd", 254}
//      }},
//     {"payload", {
//        {"custom_cmd", 10000},
//        {"custom_data", {
//           {"reset", 1}
//         }}
//      }}
//   };

//   socket_->post_sending_msg_to_handler(jsonObject.dump());
// }

bool Usi::read_json_from_file(const std::string &filename, nlohmann::json &jsonData)
{
  std::ifstream file(filename); // Replace with your actual filename
  if (!file.is_open()) {
    std::cerr << "Error opening file." << std::endl;
    return 0;
  }

  try
  {
    jsonData.clear();
    file >> jsonData; // Read JSON data from the file
    file.close(); // Close the file
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error reading JSON: " << e.what() << std::endl;
    file.close(); // Close the file
    return 0;
  }

  return 1;
}

void Usi::identify_me()
{
  nlohmann::json cotent = {
    {"clientlocation", "full2dmap01"},
    {"clientname", "DV8"},
    {"clienttype", 1},
    {"cmd", 6},
    {"type", 1}
  };

  nlohmann::json header = {
    {"header", cotent}
  };

  std::string msg = header.dump();

  // Send the message through websocket
  socket_->post_sending_msg_to_handler(msg);
}

void Usi::test_robot()
{
  nlohmann::json jsonObject = {
    {"header", {
       {"clientname", "DV8"},
       {"cmd", 23},
       {"dest_clientname", "cag_vacuum_amr"},
       {"requestid", 11},
       {"subcmd", 254}
     }},
    {"payload", {
       {"custom_cmd", 10000},
       {"custom_data", {
          {"reset", 1}
        }}
     }}
  };

  socket_->post_sending_msg_to_handler(jsonObject.dump());
}

// void Usi::start_mission(const std::string &filename, int mission_id)
// {

//   nlohmann::json mission;
//   read_json_from_file(filename, mission);

//   if(mission_id != -1000)
//     mission["mission_id"] = mission_id;

//   nlohmann::json jsonObject = {
//     {"header", {
//        {"clientname", "isera2-hawkeye1"},
//        {"cmd", 13},
//        {"dest_clientname", "CAG_1"},
//        {"requestid", 35},
//        {"subcmd", 254}
//      }},
//     {"payload", mission}
//   };

//   // std::cout<<jsonObject.dump()<<std::endl;
//   socket_->post_sending_msg_to_handler(jsonObject.dump());
// }

void Usi::post_received_msg_to_handler(const std::string &msg)
{
  boost::asio::post(io_context_, boost::bind(&Usi::handle_received_msg,this,msg));
}

void Usi::handle_ros_event(const boost::system::error_code &error)
{
  if (error)
  {
    printf("ASIO-ROS main loop Error: %s\n", error.message().c_str());
    stop_asio();
    return;
  }
  if (ros::ok())
  {
    ros::spinOnce();
    // Set the timer for next trigger
    timer_.expires_from_now(boost::posix_time::milliseconds(timer_duration_));
    timer_.async_wait(boost::bind(&Usi::handle_ros_event, this, boost::asio::placeholders::error));
  }
  else
  {
    stop_asio();
  }
}

void Usi::ros_msg_callback(const std_msgs::String & msg)
{
  // std::cout<<msg.data<<std::endl;
  if (msg.data=="identify_me")
  {
    identify_me();
  }
  else if(msg.data == "test")
  {
    test_robot();
  }
  else if(msg.data == "mission")
  {
    start_mission("/home/sp/Downloads/cag_room_toilet_bowl_entry_mission.txt");
  }
}

void Usi::stop_asio()
{
  work_.reset();
  io_context_.stop();
}

void Usi::handle_received_msg(std::string msg)
{
  std::cout<<"Mainthread receive at: "<<std::this_thread::get_id()<<std::endl;

  nlohmann::json jobject = nlohmann::json::parse(msg);

  if(jobject.contains("header") && jobject["header"].contains("response"))
  {
    if(jobject["header"]["response"] == 12 || jobject["header"]["response"] == 13)
    {
      std::cout<<"Mainthread receive at: "<<std::this_thread::get_id()<<" "<<msg<<std::endl;
      // arm_task_ctrl::UgvMissionState msg;
      // msg.mission_id = jobject["payload"]["mission_id"];
      // msg.mission_state = jobject["payload"]["mission_status"];
      // mission_finished_pub_.publish(msg);
    }
  }
}
