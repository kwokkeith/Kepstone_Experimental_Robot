#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <thread>
#include <ros/ros.h>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <websocket_comm/json.hpp>
#include <queue>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
namespace ssl = boost::asio::ssl;
using tcp = net::ip::tcp;

class Usi;

class WebSocket
{
public:
  WebSocket(ssl::context &ctx, Usi *usi);
  ~WebSocket();
  void connect();
  void disconnect();
  void post_sending_msg_to_handler(const std::string &msg);

private:
  void worker_close();
  void worker_receive_msg();
  void worker_send_msg(std::string msg);
  void worker_write_data();

private:
  Usi* usi_;
  boost::asio::io_context io_ctx_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_;
  std::thread thread_;

  tcp::resolver resolver_;
  websocket::stream<ssl::stream<tcp::socket>> ws_;
  beast::flat_buffer read_buffer_;
  std::atomic<bool> is_writing_;
  std::queue<std::string> write_buffer_;
};

#endif // WEBSOCKET_H
