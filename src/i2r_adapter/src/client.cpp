#include "i2r_adapter/client.hpp"

namespace mrccc_utils {
namespace websocket_client {

using websocketpp::connection_hdl;
using client = websocketpp::client<websocketpp::config::asio_tls_client>;

// TLS Initialization for secure WebSocket connection
connection_metadata::context_ptr connection_metadata::on_tls_init(websocketpp::connection_hdl) {
    connection_metadata::context_ptr ctx = std::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::sslv23);
    try {
        ctx->set_options(boost::asio::ssl::context::default_workarounds |
                         boost::asio::ssl::context::no_sslv2 |
                         boost::asio::ssl::context::no_sslv3 |
                         boost::asio::ssl::context::single_dh_use);
    } catch (std::exception& e) {
        std::cout << "Error in TLS context: " <<  e.what() << std::endl;
    }
    return ctx;
}

// WebSocket event handlers
void connection_metadata::on_open(client* c, websocketpp::connection_hdl hdl) {
    m_status = "Open";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
    std::cout << "WebSocket connection opened" << std::endl;
}

void connection_metadata::on_fail(client* c, websocketpp::connection_hdl hdl) {
    m_status = "Failed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    m_server = con->get_response_header("Server");
    m_error_reason = con->get_ec().message();
    std::cout << "WebSocket connection failed: " << m_error_reason.c_str() << std::endl;
}

void connection_metadata::on_close(client* c, websocketpp::connection_hdl hdl) {
    m_status = "Closed";
    client::connection_ptr con = c->get_con_from_hdl(hdl);
    std::stringstream s;
    s << "Close code: " << con->get_remote_close_code() << " (" 
      << websocketpp::close::status::get_string(con->get_remote_close_code()) 
      << "), Close reason: " << con->get_remote_close_reason();
    m_error_reason = s.str();
    std::cout << "WebSocket connection closed" << std::endl;
}

void connection_metadata::on_message(websocketpp::connection_hdl, client::message_ptr msg) {
    std::lock_guard<std::mutex> lck (_mtx);
    // rmf_fleet_msgs::msg::FleetState fs;

    // Needs mrccc_utils package from I2R
    /* 
    mrccc_utils::feedback_parser::RobotStateUpdate(
        msg->get_payload(), 
        fs,
        path_compeletion_status);
    */
}

// WebSocket Endpoint implementation
int websocket_endpoint::connect(const std::string& uri) {
    websocketpp::lib::error_code ec;
    m_endpoint.set_tls_init_handler(connection_metadata::on_tls_init);
    client::connection_ptr con = m_endpoint.get_connection(uri, ec);

    if (ec) {
        std::cout << "WebSocket connection error: " << ec.message().c_str() << std::endl;
        return -1;
    }

    int new_id = m_next_id++;
    // Creates a new connection_metadata object
    connection_metadata::ptr metadata_ptr = 
        websocketpp::lib::make_shared<connection_metadata>(new_id, con->get_handle(), uri);
    
    // New metadata object stored in m_connection_list
    m_connection_list[new_id] = metadata_ptr;

    con->set_open_handler(websocketpp::lib::bind(
        &connection_metadata::on_open, 
        metadata_ptr, 
        &m_endpoint, 
        websocketpp::lib::placeholders::_1));
    con->set_fail_handler(websocketpp::lib::bind(
        &connection_metadata::on_fail, 
        metadata_ptr, 
        &m_endpoint, 
        websocketpp::lib::placeholders::_1));
    con->set_close_handler(websocketpp::lib::bind(
        &connection_metadata::on_close, 
        metadata_ptr, 
        &m_endpoint, 
        websocketpp::lib::placeholders::_1));
    con->set_message_handler(websocketpp::lib::bind(
        &connection_metadata::on_message, 
        metadata_ptr, 
        websocketpp::lib::placeholders::_1, 
        websocketpp::lib::placeholders::_2));
    
    m_endpoint.connect(con);
    
    return new_id;
}

void websocket_endpoint::close(int id, websocketpp::close::status::value code, const std::string& reason) {
    websocketpp::lib::error_code ec;
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        std::cout << "No connection found with id " << std::to_string(id) << std::endl;
        return;
    }
    m_endpoint.close(metadata_it->second->get_hdl(), code, reason, ec);
    if (ec) {
        std::cout << "Error closing WebSocket: " << ec.message().c_str() << std::endl;
    }
}

void websocket_endpoint::send(int id, std::string message, websocketpp::lib::error_code& e) {
    websocketpp::lib::error_code ec;
    std::cout<<message<<std::endl;
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        std::cout << "> No connection found with id " << id << std::endl;
        return;
    }
    
    m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cout << "> Error sending message: " << ec.message() << std::endl;
        e = ec;
        return;
    }
    
    metadata_it->second->record_sent_message(message);
}


void websocket_endpoint::send(int id, const std::string message) {
    websocketpp::lib::error_code ec;
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        std::cout << "No connection found with id " << std::to_string(id) << std::endl;
        return;
    }
    m_endpoint.send(metadata_it->second->get_hdl(), message, websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cout << "Error sending WebSocket message: " << ec.message().c_str() << std::endl;
        return;
    }
    metadata_it->second->record_sent_message(message);
    std::cout << "WebSocket message sent" << std::endl;
}

connection_metadata::ptr websocket_endpoint::get_metadata(int id) const {
    auto metadata_it = m_connection_list.find(id);
    if (metadata_it == m_connection_list.end()) {
        return connection_metadata::ptr();
    }
    return metadata_it->second;
}

} // namespace websocket_client
} // namespace mrccc_utils
