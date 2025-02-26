#include <ros/ros.h>
#include <std_msgs/String.h>
#include <i2r_adapter/SendJson.h> 
#include "i2r_adapter/client.hpp"

std::shared_ptr<mrccc_utils::websocket_client::websocket_endpoint> ws_client;
int connection_id = -1; // Default WebSocket connection id
websocketpp::lib::error_code ec;

// Function for WebSocket initialization
void wss_client_init()
{
    websocketpp::lib::error_code _ec;

    // Establish WebSocket connection
    std::string websocket_url = "wss://0.0.0.0:5000";
    connection_id = ws_client->connect(websocket_url);

    if (connection_id == -1) {
        ROS_ERROR("Failed to establish WebSocket connection.");
        return;
    }

    ROS_INFO("WebSocket connected with ID: %d", connection_id);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Allow time for connection setup

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


// Function to ensure WebSocket connection is established
void check_init_completion()
{
    while (ec) {
        ROS_WARN("Reattempting WebSocket connection...");
        wss_client_init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

// Not needed because only one robot
// // Thread-safe lock function
// std::mutex _mutex;
// std::unique_lock<std::mutex> lock()
// {
//     std::unique_lock<std::mutex> l(_mutex, std::defer_lock);
//     while (!l.try_lock()) {
//         // Intentionally busy wait
//     }
//     return l;
// }

// Service callback to send JSON messages via WebSocket
// Example usage: 
// rosservice call /send_json '{"json_message": "{\"command\": \"move\", \"speed\": 1.5}"}'
// bool sendJsonCallback(i2r_adapter::SendJson::Request &req, i2r_adapter::SendJson::Response &res) {
//     if (connection_id == -1) {
//         res.success = false;
//         res.response_message = "WebSocket is not connected!";
//         ROS_ERROR("%s", res.response_message.c_str());
//         return true;
//     }

//     ws_client->send(connection_id, req.json_message);

//     return true;
// }

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
