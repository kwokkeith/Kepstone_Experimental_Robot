#include "websocket_comm/websocket.h"
#include "websocket_comm/usi.h"

WebSocket::WebSocket(ssl::context &ctx, Usi *usi):
  usi_(usi),
  work_(boost::asio::make_work_guard(io_ctx_)),
  resolver_(io_ctx_),
  ws_(io_ctx_, ctx),
  is_writing_(false)
{

}

WebSocket::~WebSocket()
{

}

void WebSocket::connect()
{
  std::string host = "192.168.0.150";
  auto const  port = "5100";

  // Look up the domain name
  auto const results = resolver_.resolve(host, port);

  // Make the connection on the IP address we get from a lookup
  auto ep = net::connect(beast::get_lowest_layer(ws_), results);

  // Set SNI Hostname (many hosts need this to handshake successfully)
  if(! SSL_set_tlsext_host_name(ws_.next_layer().native_handle(), host.c_str()))
    throw beast::system_error(
        beast::error_code(
          static_cast<int>(::ERR_get_error()),
          net::error::get_ssl_category()),
        "Failed to set SNI Hostname");

  // Update the host_ string. This will provide the value of the
  // Host HTTP header during the WebSocket handshake.
  // See https://tools.ietf.org/html/rfc7230#section-5.4
  host += ':' + std::to_string(ep.port());

  // Perform the SSL handshake
  ws_.next_layer().handshake(ssl::stream_base::client);

  // Set a decorator to change the User-Agent of the handshake
  ws_.set_option(websocket::stream_base::decorator(
                   [](websocket::request_type& req)
                 {
                   req.set(http::field::user_agent,
                   std::string(BOOST_BEAST_VERSION_STRING) +
                   " websocket-client-coro");
                 }));

  // Perform the websocket handshake
  ws_.handshake(host, "/");

  // Post an event to the event queue to keep it from dying
  boost::asio::post(io_ctx_, std::bind(&WebSocket::worker_receive_msg, this));
  thread_ = std::thread([this] () {
    io_ctx_.run();
  });
}

void WebSocket::disconnect()
{
  boost::asio::post(io_ctx_, boost::bind(&WebSocket::worker_close, this));
  if (thread_.joinable())
    thread_.join();

  try
  {
    ws_.close(websocket::close_code::normal);
  }
  catch(const boost::system::system_error &e)
  {
    std::cout<<e.what()<<std::endl;
  }
}

void WebSocket::post_sending_msg_to_handler(const std::string &msg)
{
  std::cout<<"main send at: "<<std::this_thread::get_id()<<std::endl;
  boost::asio::post(io_ctx_, boost::bind(&WebSocket::worker_send_msg, this, msg));
}

void WebSocket::worker_receive_msg()
{
  //std::cout<<"Worker call receive at: "<<std::this_thread::get_id()<<std::endl;
  ws_.async_read(
        read_buffer_,
        [this] ( boost::system::error_code error, size_t bytes_transferred)
  {
    //std::cout<<"Worker receive at: "<<std::this_thread::get_id()<<std::endl;
    boost::ignore_unused(bytes_transferred);

    if (error)
    {
      printf("websocket error in read: %s\n", error.message().c_str());
      exit(-1);
    }
    else
    {
      boost::asio::const_buffer readable_bytes = beast::buffers_front(read_buffer_.data());

      // Convert to a string
      std::string msg(boost::asio::buffer_cast<const char*>(readable_bytes),
                      boost::asio::buffer_size(readable_bytes));

      usi_->post_received_msg_to_handler(msg);
    }

    read_buffer_.clear();
    worker_receive_msg();
  });
}

void WebSocket::worker_send_msg(std::string msg)
{
  write_buffer_.push(msg);
  if (is_writing_)
    return;
  else
    worker_write_data();
}

void WebSocket::worker_write_data()
{
  is_writing_ = true;
  std::cout<<"Worker start write at: "<<std::this_thread::get_id()<<std::endl;
  ws_.async_write(
        boost::asio::buffer(write_buffer_.front()),
        [this] (boost::system::error_code error,
        std::size_t bytes_transferred)
  {
    boost::ignore_unused(bytes_transferred);
    if (error)
    {
      printf("websocket error in write: %s\n", error.message().c_str());
      exit(-1);
    }

    std::cout<<"Worker finish write at: "<<std::this_thread::get_id()<<std::endl;

    write_buffer_.pop();
    if(!write_buffer_.empty())
      worker_write_data();
    else
      is_writing_ = false;
  }
  );
}

void WebSocket::worker_close()
{
  work_.reset();
  io_ctx_.stop();
  std::cout<<"Socket closed in thread: "<<std::this_thread::get_id()<<std::endl;
}
