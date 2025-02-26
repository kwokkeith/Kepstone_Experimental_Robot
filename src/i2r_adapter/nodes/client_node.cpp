#include <ros/ros.h>
#include <std_msgs/String.h>
#include <i2r_adapter/SendJson.h> 
#include <i2r_adapter/mission_gen.hpp>
#include "i2r_adapter/client.hpp"

std::shared_ptr<mrccc_utils::websocket_client::websocket_endpoint> ws_client;
int connection_id = -1; // Default WebSocket connection id
websocketpp::lib::error_code ec;

// Function for WebSocket initialization
void wss_client_init()
{
    websocketpp::lib::error_code _ec;

    // Establish WebSocket connection
    std::string websocket_url = "wss://192.168.0.150:5100";

    // The SSL context is required, and holds certificates
    ssl::context ctx{ssl::context::tlsv12_client};

    // This holds the root certificate used for verification
    std::string certification_file =  "/home/keith/Documents/Capstone/Kepstone_Experimental_Robot/src/i2r_adapter/lic/default_gui1_pem.crt";
    std::string key_file = "src/i2r_adapter/lic/default_gui1_pem.key";

    ctx.use_certificate_file(certification_file, ssl::context::pem);
    ctx.set_password_callback([](std::size_t,
                                boost::asio::ssl::context_base::password_purpose)
    {
        return "pass_default_gui1";
    });
    ctx.use_rsa_private_key_file(key_file, ssl::context::pem);

    WebSocket
    connection_id = ws_client->connect(websocket_url);

    if (connection_id == -1) {
        ROS_ERROR("Failed to establish WebSocket connection.");
        return;
    }

    ROS_INFO("WebSocket connected with ID: %d", connection_id);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow time for connection setup

    std::string idme_cmd = mrccc_utils::mission_gen::identifyMe();
    ws_client->send(connection_id, idme_cmd, _ec);

    auto metadata = ws_client->get_metadata(connection_id);
    int retries = 5;
    while (metadata && metadata->get_status() != "Open" && retries > 0) {
        ROS_WARN("WebSocket is not open yet, retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        metadata = ws_client->get_metadata(connection_id);
        retries--;
    }

    if (!metadata || metadata->get_status() != "Open") {
        ROS_ERROR("WebSocket failed to open after multiple retries.");
        ws_client.reset();
        return;
    }

    ROS_INFO("WebSocket is now OPEN.");
}

void connect()
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



// Function to ensure WebSocket connection is established
void check_init_completion()
{
    while (ec) {
        ROS_WARN("Reattempting WebSocket connection...");
        wss_client_init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}


bool sendJsonCallback(i2r_adapter::SendJson::Request &req, i2r_adapter::SendJson::Response &res) {
    if (connection_id == -1) {
        res.success = false;
        res.response_message = "WebSocket is not connected!";
        ROS_ERROR("%s", res.response_message.c_str());
        return true;
    }

    auto metadata = ws_client->get_metadata(connection_id);
    if (!metadata) {
        res.success = false;
        res.response_message = "WebSocket metadata not found!";
        ROS_ERROR("%s", res.response_message.c_str());
        return true;
    }

    if (metadata->get_status() != "Open") {
        res.success = false;
        res.response_message = "WebSocket is not in OPEN state!";
        ROS_ERROR("%s", res.response_message.c_str());
        return true;
    }

    websocketpp::lib::error_code ec;
    ws_client->send(connection_id, req.json_message, ec);

    if (ec) {
        res.success = false;
        res.response_message = "Failed to send message: " + ec.message();
        ROS_ERROR("%s", res.response_message.c_str());
    } else {
        res.success = true;
        res.response_message = "Message sent successfully.";
        ROS_INFO("Sent JSON: %s", req.json_message.c_str());
    }

    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "websocket_client_node");
    ros::NodeHandle nh;

    // Create WebSocket client instance
    ws_client = std::make_shared<mrccc_utils::websocket_client::websocket_endpoint>();

    // Initialize WebSocket connection
    wss_client_init();

    // Ensure WebSocket initialization completes
    check_init_completion();

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("send_json", sendJsonCallback);
    ROS_INFO("WebSocket send_json service is ready.");

    ros::spin(); // Keep the node running

    // Close WebSocket before shutdown
    ws_client->close(connection_id, websocketpp::close::status::normal, "ROS shutdown");

    return 0;
}
