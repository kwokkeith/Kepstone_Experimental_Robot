#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>

#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/functional.hpp>

#include <jsoncpp/json/json.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <rosconsole/macros_generated.h>
#include <ros/ros.h>
#include <mutex>
#include <memory>

namespace ssl = boost::asio::ssl;

namespace mrccc_utils {
namespace websocket_client {

using websocketpp::connection_hdl;
using client = websocketpp::client<websocketpp::config::asio_tls_client>;

class connection_metadata {
public:
    using ptr = std::shared_ptr<connection_metadata>;
    using context_ptr = std::shared_ptr<boost::asio::ssl::context>;

    static context_ptr on_tls_init(websocketpp::connection_hdl);
    void on_open(client* c, websocketpp::connection_hdl hdl);
    void on_fail(client* c, websocketpp::connection_hdl hdl);
    void on_close(client* c, websocketpp::connection_hdl hdl);
    void on_message(websocketpp::connection_hdl, client::message_ptr msg);

    connection_metadata(int id, websocketpp::connection_hdl hdl, const std::string& uri)
      : 
      m_id(id), 
      m_hdl(hdl), 
      m_uri(uri), 
      m_status("Connecting"), 
      m_server("N/A") {}

    connection_hdl get_hdl() const { return m_hdl; }
    std::string get_status() const { return m_status; }
    void record_sent_message(std::string message) {
       
        m_messages.push_back(">> " + message);
    }
    std::vector<std::string> m_messages;

private:
    int m_id;
    connection_hdl m_hdl;
    std::string m_uri;
    std::string m_status;
    std::string m_server;
    std::string m_error_reason;
    std::mutex _mtx;
};

class websocket_endpoint {
public:
    websocket_endpoint() {
        m_endpoint.init_asio();
        m_endpoint.set_tls_init_handler(connection_metadata::on_tls_init);
    }
    int connect(const std::string& uri);
    void close(int id, websocketpp::close::status::value code, const std::string& reason);
    void send(int id, std::string message, websocketpp::lib::error_code& e);
    void send(int id, const std::string message);

    connection_metadata::ptr get_metadata(int id) const;

    typedef std::map<int, connection_metadata::ptr> con_list;

    con_list m_connection_list;

private:
    client m_endpoint;
    // std::map<int, connection_metadata::ptr> m_connection_list;
    int m_next_id = 0;
};

} // namespace websocket_client
} // namespace mrccc_utils

#endif // WEBSOCKET_CLIENT_H
