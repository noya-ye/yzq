#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ground_station_msgs/msg/ground_command.hpp"
#include "ground_station_msgs/msg/mission_plan.hpp"
#include "ground_station_msgs/msg/task_status.hpp"

using namespace std::chrono_literals;

class UdpGroundStationBridge : public rclcpp::Node
{
public:
  UdpGroundStationBridge()
  : Node("udp_ground_station_bridge")
  {
    bind_ip_ =
      declare_parameter<std::string>(
        "bind_ip",
        "0.0.0.0");

    bind_port_ =
      declare_parameter<int>(
        "bind_port",
        9000);

    remote_ip_ =
      declare_parameter<std::string>(
        "remote_ip",
        "192.168.31.187");

    remote_port_ =
      declare_parameter<int>(
        "remote_port",
        9001);

    print_rx_ =
      declare_parameter<bool>(
        "print_rx",
        true);

    print_tx_ =
      declare_parameter<bool>(
        "print_tx",
        true);

    command_pub_ =
      create_publisher<
        ground_station_msgs::msg::GroundCommand>(
        "/ground_station/cmd_parsed",
        10);

    status_sub_ =
      create_subscription<
        ground_station_msgs::msg::TaskStatus>(
        "/ground_station/task_status",
        10,
        std::bind(
          &UdpGroundStationBridge::statusCallback,
          this,
          std::placeholders::_1));

    mission_plan_sub_ =
      create_subscription<
        ground_station_msgs::msg::MissionPlan>(
        "/ground_station/mission_plan",
        rclcpp::QoS(1).reliable(),
        std::bind(
          &UdpGroundStationBridge::missionPlanCallback,
          this,
          std::placeholders::_1));

    openSocket();

    receive_timer_ =
      create_wall_timer(
        20ms,
        std::bind(
          &UdpGroundStationBridge::receiveUdp,
          this));

    RCLCPP_INFO(
      get_logger(),
      "[UDP] listen=%s:%d remote=%s:%d",
      bind_ip_.c_str(),
      bind_port_,
      remote_ip_.c_str(),
      remote_port_);
  }

  ~UdpGroundStationBridge() override
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

private:
  void openSocket()
  {
    socket_fd_ =
      socket(
        AF_INET,
        SOCK_DGRAM,
        0);

    if (socket_fd_ < 0) {
      throw std::runtime_error(
        "failed to create UDP socket");
    }

    const int flags =
      fcntl(
        socket_fd_,
        F_GETFL,
        0);

    fcntl(
      socket_fd_,
      F_SETFL,
      flags | O_NONBLOCK);

    sockaddr_in bind_address{};
    bind_address.sin_family = AF_INET;
    bind_address.sin_port =
      htons(
        static_cast<uint16_t>(
          bind_port_));

    if (inet_pton(
          AF_INET,
          bind_ip_.c_str(),
          &bind_address.sin_addr) != 1) {
      throw std::runtime_error(
        "invalid bind_ip");
    }

    if (bind(
          socket_fd_,
          reinterpret_cast<sockaddr*>(
            &bind_address),
          sizeof(bind_address)) < 0) {
      throw std::runtime_error(
        std::string("UDP bind failed: ") +
        std::strerror(errno));
    }

    remote_address_.sin_family = AF_INET;
    remote_address_.sin_port =
      htons(
        static_cast<uint16_t>(
          remote_port_));

    if (inet_pton(
          AF_INET,
          remote_ip_.c_str(),
          &remote_address_.sin_addr) != 1) {
      throw std::runtime_error(
        "invalid remote_ip");
    }
  }

  static std::string trim(
    const std::string& text)
  {
    const auto begin =
      std::find_if_not(
        text.begin(),
        text.end(),
        [](unsigned char c) {
          return std::isspace(c);
        });

    const auto end =
      std::find_if_not(
        text.rbegin(),
        text.rend(),
        [](unsigned char c) {
          return std::isspace(c);
        }).base();

    if (begin >= end) {
      return "";
    }

    return std::string(begin, end);
  }

  static std::string upper(
    std::string text)
  {
    std::transform(
      text.begin(),
      text.end(),
      text.begin(),
      [](unsigned char c) {
        return static_cast<char>(
          std::toupper(c));
      });

    return text;
  }

  static std::string escapeJson(
    const std::string& text)
  {
    std::string result;
    result.reserve(text.size());

    for (const char c : text) {
      if (c == '\\' || c == '"') {
        result.push_back('\\');
      }

      result.push_back(c);
    }

    return result;
  }

  void receiveUdp()
  {
    char buffer[2048];

    while (true) {
      sockaddr_in sender_address{};
      socklen_t sender_length =
        sizeof(sender_address);

      const ssize_t received =
        recvfrom(
          socket_fd_,
          buffer,
          sizeof(buffer) - 1,
          0,
          reinterpret_cast<sockaddr*>(
            &sender_address),
          &sender_length);

      if (received < 0) {
        if (errno == EAGAIN ||
            errno == EWOULDBLOCK) {
          return;
        }

        RCLCPP_ERROR(
          get_logger(),
          "[UDP RX] recvfrom failed: %s",
          std::strerror(errno));

        return;
      }

      buffer[received] = '\0';

      const std::string text =
        trim(
          std::string(
            buffer,
            static_cast<std::size_t>(
              received)));

      if (text.empty()) {
        continue;
      }

      if (print_rx_) {
        RCLCPP_INFO(
          get_logger(),
          "[UDP RX] %s",
          text.c_str());
      }

      publishCommand(text);
    }
  }

  void publishCommand(
    const std::string& text)
  {
    ground_station_msgs::msg::GroundCommand msg;

    msg.raw = text;
    msg.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_UNKNOWN;

    msg.has_goal = false;
    msg.x = 0.0f;
    msg.y = 0.0f;
    msg.z = 0.0f;

    std::istringstream stream(text);

    std::string command;
    stream >> command;

    command = upper(command);

    if (command == "START") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_START;
    } else if (command == "PAUSE") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_PAUSE;
    } else if (command == "RESUME") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_RESUME;
    } else if (command == "RTL") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_RTL;
    } else if (command == "LAND") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_LAND;
    } else if (command == "RESET") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_RESET;
    } else if (command == "CLEAR_TRACK") {
      msg.cmd_type =
        ground_station_msgs::msg::GroundCommand::
        CMD_CLEAR_TRACK;
    } else if (command == "GOTO") {
      float x = 0.0f;
      float y = 0.0f;
      float z = 0.0f;

      if (stream >> x >> y >> z) {
        msg.cmd_type =
          ground_station_msgs::msg::GroundCommand::
          CMD_GOTO;

        msg.has_goal = true;
        msg.x = x;
        msg.y = y;
        msg.z = z;
      }
    }

    command_pub_->publish(msg);
  }

  void statusCallback(
    const ground_station_msgs::msg::TaskStatus::SharedPtr msg)
  {
    std::ostringstream json;

    json
      << "{"
      << "\"type\":\"status\","
      << "\"task\":\""
      << escapeJson(msg->task_name)
      << "\","
      << "\"current_wp\":"
      << msg->current_wp
      << ","
      << "\"total_wp\":"
      << msg->total_wp
      << ","
      << "\"mission_done\":"
      << (msg->mission_done ? "true" : "false")
      << ","
      << "\"target_type\":"
      << msg->target_type
      << ","
      << "\"target_name\":\""
      << escapeJson(msg->target_name)
      << "\""
      << "}";

    sendUdp(json.str());
  }

  void missionPlanCallback(
    const ground_station_msgs::msg::MissionPlan::SharedPtr msg)
  {
    std::ostringstream json;

    json
      << "{"
      << "\"type\":\"plan\","
      << "\"plan_id\":"
      << msg->plan_id
      << ","
      << "\"route\":[";

    for (std::size_t i = 0;
         i < msg->route_cells.size();
         ++i) {
      if (i > 0) {
        json << ",";
      }

      json
        << "\""
        << escapeJson(
          msg->route_cells[i])
        << "\"";
    }

    json
      << "]"
      << "}";

    sendUdp(json.str());
  }

  void sendUdp(
    const std::string& text)
  {
    const ssize_t sent =
      sendto(
        socket_fd_,
        text.data(),
        text.size(),
        0,
        reinterpret_cast<const sockaddr*>(
          &remote_address_),
        sizeof(remote_address_));

    if (sent < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "[UDP TX] sendto failed: %s",
        std::strerror(errno));

      return;
    }

    if (print_tx_) {
      RCLCPP_INFO(
        get_logger(),
        "[UDP TX] %s",
        text.c_str());
    }
  }

private:
  std::string bind_ip_;
  int bind_port_{9000};

  std::string remote_ip_;
  int remote_port_{9001};

  bool print_rx_{true};
  bool print_tx_{true};

  int socket_fd_{-1};

  sockaddr_in remote_address_{};

  rclcpp::Publisher<
    ground_station_msgs::msg::GroundCommand>::SharedPtr
    command_pub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::TaskStatus>::SharedPtr
    status_sub_;

  rclcpp::Subscription<
    ground_station_msgs::msg::MissionPlan>::SharedPtr
    mission_plan_sub_;

  rclcpp::TimerBase::SharedPtr
    receive_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      UdpGroundStationBridge>());

  rclcpp::shutdown();

  return 0;
}