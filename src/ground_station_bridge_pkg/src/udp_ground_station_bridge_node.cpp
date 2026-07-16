#include "ground_station_bridge_pkg/udp_ground_station_bridge_node.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <sstream>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ground_station_bridge_pkg
{

UdpGroundStationBridgeNode::UdpGroundStationBridgeNode()
: Node("udp_ground_station_bridge_node")
{
  udp_bind_ip_ =
    declare_parameter<std::string>(
      "udp_bind_ip", "0.0.0.0");

  udp_bind_port_ =
    declare_parameter<int>(
      "udp_bind_port", 9000);

  udp_remote_ip_ =
    declare_parameter<std::string>(
      "udp_remote_ip", "192.168.43.50");

  udp_remote_port_ =
    declare_parameter<int>(
      "udp_remote_port", 9001);

  print_tx_ =
    declare_parameter<bool>("print_tx", true);

  print_rx_ =
    declare_parameter<bool>("print_rx", true);

  raw_cmd_topic_ =
    declare_parameter<std::string>(
      "raw_cmd_topic",
      "/ground_station/cmd");

  parsed_cmd_topic_ =
    declare_parameter<std::string>(
      "parsed_cmd_topic",
      "/ground_station/cmd_parsed");

  mission_plan_topic_ =
    declare_parameter<std::string>(
      "mission_plan_topic",
      "/ground_station/mission_plan");

  task_status_topic_ =
    declare_parameter<std::string>(
      "task_status_topic",
      "/ground_station/task_status");

  animal_info_topic_ =
    declare_parameter<std::string>(
      "animal_info_topic",
      "/ground_station/animal_info");

  mission_result_topic_ =
    declare_parameter<std::string>(
      "mission_result_topic",
      "/ground_station/mission_result");

  if (!udp_init()) {
    RCLCPP_ERROR(get_logger(), "UDP init failed");
  }

  parsed_cmd_pub_ =
    create_publisher<
      ground_station_msgs::msg::GroundCommand>(
      parsed_cmd_topic_,
      10);

  raw_cmd_sub_ =
    create_subscription<std_msgs::msg::String>(
      raw_cmd_topic_,
      10,
      std::bind(
        &UdpGroundStationBridgeNode::raw_cmd_cb,
        this,
        std::placeholders::_1));

  mission_plan_sub_ =
    create_subscription<
      ground_station_msgs::msg::MissionPlan>(
      mission_plan_topic_,
      rclcpp::QoS(1).reliable(),
      std::bind(
        &UdpGroundStationBridgeNode::mission_plan_cb,
        this,
        std::placeholders::_1));

  task_status_sub_ =
    create_subscription<
      ground_station_msgs::msg::TaskStatus>(
      task_status_topic_,
      10,
      std::bind(
        &UdpGroundStationBridgeNode::task_status_cb,
        this,
        std::placeholders::_1));

  animal_info_sub_ =
    create_subscription<
      ground_station_msgs::msg::AnimalInfo>(
      animal_info_topic_,
      10,
      std::bind(
        &UdpGroundStationBridgeNode::animal_info_cb,
        this,
        std::placeholders::_1));

  mission_result_sub_ =
    create_subscription<
      ground_station_msgs::msg::MissionResult>(
      mission_result_topic_,
      10,
      std::bind(
        &UdpGroundStationBridgeNode::mission_result_cb,
        this,
        std::placeholders::_1));

  rx_timer_ =
    create_wall_timer(
      50ms,
      std::bind(
        &UdpGroundStationBridgeNode::rx_timer_cb,
        this));

  RCLCPP_INFO(
    get_logger(),
    "[UDP] bind=%s:%d remote=%s:%d",
    udp_bind_ip_.c_str(),
    udp_bind_port_,
    udp_remote_ip_.c_str(),
    udp_remote_port_);
}

UdpGroundStationBridgeNode::~UdpGroundStationBridgeNode()
{
  udp_close();
}

bool UdpGroundStationBridgeNode::udp_init()
{
  udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);

  if (udp_fd_ < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "socket failed: %s",
      std::strerror(errno));

    return false;
  }

  int reuse = 1;

  ::setsockopt(
    udp_fd_,
    SOL_SOCKET,
    SO_REUSEADDR,
    &reuse,
    sizeof(reuse));

  const int flags =
    ::fcntl(udp_fd_, F_GETFL, 0);

  if (flags >= 0) {
    ::fcntl(
      udp_fd_,
      F_SETFL,
      flags | O_NONBLOCK);
  }

  local_addr_.sin_family = AF_INET;
  local_addr_.sin_port =
    htons(
      static_cast<uint16_t>(
        udp_bind_port_));

  if (::inet_pton(
        AF_INET,
        udp_bind_ip_.c_str(),
        &local_addr_.sin_addr) != 1) {
    RCLCPP_ERROR(
      get_logger(),
      "invalid bind IP: %s",
      udp_bind_ip_.c_str());

    udp_close();
    return false;
  }

  if (::bind(
        udp_fd_,
        reinterpret_cast<sockaddr*>(
          &local_addr_),
        sizeof(local_addr_)) < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "bind failed: %s",
      std::strerror(errno));

    udp_close();
    return false;
  }

  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port =
    htons(
      static_cast<uint16_t>(
        udp_remote_port_));

  if (::inet_pton(
        AF_INET,
        udp_remote_ip_.c_str(),
        &remote_addr_.sin_addr) != 1) {
    RCLCPP_ERROR(
      get_logger(),
      "invalid remote IP: %s",
      udp_remote_ip_.c_str());

    udp_close();
    return false;
  }

  return true;
}

void UdpGroundStationBridgeNode::udp_close()
{
  if (udp_fd_ >= 0) {
    ::close(udp_fd_);
    udp_fd_ = -1;
  }
}

void UdpGroundStationBridgeNode::send_udp(
  const std::string& data)
{
  if (udp_fd_ < 0) {
    return;
  }

  const ssize_t sent =
    ::sendto(
      udp_fd_,
      data.data(),
      data.size(),
      0,
      reinterpret_cast<sockaddr*>(
        &remote_addr_),
      sizeof(remote_addr_));

  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "UDP send failed: %s",
      std::strerror(errno));

    return;
  }

  if (print_tx_) {
    RCLCPP_INFO(
      get_logger(),
      "[UDP TX] %s",
      data.c_str());
  }
}

void UdpGroundStationBridgeNode::mission_plan_cb(
  const ground_station_msgs::msg::MissionPlan::SharedPtr msg)
{
  std::ostringstream json;

  json
    << "{"
    << "\"type\":\"plan\","
    << "\"plan_id\":" << msg->plan_id << ","
    << "\"route\":[";

  for (std::size_t i = 0;
       i < msg->route_cells.size();
       ++i) {
    if (i > 0) {
      json << ",";
    }

    json
      << "\""
      << escape_json(msg->route_cells[i])
      << "\"";
  }

  json << "]}";

  send_udp(json.str());
}

void UdpGroundStationBridgeNode::task_status_cb(
  const ground_station_msgs::msg::TaskStatus::SharedPtr msg)
{
  std::ostringstream json;

  json
    << "{"
    << "\"type\":\"status\","
    << "\"plan_id\":" << msg->plan_id << ","
    << "\"state\":"
    << static_cast<unsigned>(msg->state) << ","
    << "\"current_index\":"
    << msg->current_index << ","
    << "\"current_cell\":\""
    << escape_json(msg->current_cell) << "\","
    << "\"mission_done\":"
    << (msg->mission_done ? "true" : "false")
    << "}";

  send_udp(json.str());
}

void UdpGroundStationBridgeNode::animal_info_cb(
  const ground_station_msgs::msg::AnimalInfo::SharedPtr msg)
{
  std::ostringstream json;

  json
    << "{"
    << "\"type\":\"animal\","
    << "\"plan_id\":" << msg->plan_id << ","
    << "\"cell\":\""
    << escape_json(msg->cell) << "\","
    << "\"animal_type\":"
    << static_cast<unsigned>(msg->animal_type) << ","
    << "\"count\":" << msg->count
    << "}";

  send_udp(json.str());
}

void UdpGroundStationBridgeNode::mission_result_cb(
  const ground_station_msgs::msg::MissionResult::SharedPtr msg)
{
  std::ostringstream json;

  json
    << "{"
    << "\"type\":\"result\","
    << "\"plan_id\":" << msg->plan_id << ","
    << "\"elephant\":" << msg->elephant_count << ","
    << "\"tiger\":" << msg->tiger_count << ","
    << "\"wolf\":" << msg->wolf_count << ","
    << "\"monkey\":" << msg->monkey_count << ","
    << "\"peacock\":" << msg->peacock_count
    << "}";

  send_udp(json.str());
}

void UdpGroundStationBridgeNode::raw_cmd_cb(
  const std_msgs::msg::String::SharedPtr msg)
{
  const auto command =
    parse_command(msg->data);

  parsed_cmd_pub_->publish(command);

  RCLCPP_INFO(
    get_logger(),
    "[CMD] type=%u no_fly=%zu",
    static_cast<unsigned>(
      command.cmd_type),
    command.no_fly_cells.size());
}

void UdpGroundStationBridgeNode::rx_timer_cb()
{
  if (udp_fd_ < 0) {
    return;
  }

  char buffer[2048];

  while (true) {
    sockaddr_in source{};
    socklen_t source_length =
      sizeof(source);

    const ssize_t size =
      ::recvfrom(
        udp_fd_,
        buffer,
        sizeof(buffer) - 1,
        0,
        reinterpret_cast<sockaddr*>(
          &source),
        &source_length);

    if (size <= 0) {
      break;
    }

    buffer[size] = '\0';

    const std::string raw(
      buffer,
      static_cast<std::size_t>(size));

    if (print_rx_) {
      RCLCPP_INFO(
        get_logger(),
        "[UDP RX] %s",
        raw.c_str());
    }

    const auto command =
      parse_command(raw);

    parsed_cmd_pub_->publish(command);
  }
}

ground_station_msgs::msg::GroundCommand
UdpGroundStationBridgeNode::parse_command(
  const std::string& raw) const
{
  ground_station_msgs::msg::GroundCommand output;

  output.cmd_type =
    ground_station_msgs::msg::GroundCommand::
    CMD_NONE;

  std::string command_name;

  /*
   * JSON格式：
   *
   * {
   *   "type":"command",
   *   "cmd":"start",
   *   "no_fly_cells":["A3B2","A3B3"]
   * }
   */
  if (raw.find('{') != std::string::npos) {
    command_name =
      upper(
        json_string(raw, "cmd"));

    output.no_fly_cells =
      json_string_array(
        raw,
        "no_fly_cells");
  } else {
    /*
     * ROS调试格式：
     *
     * START,A3B2,A3B3
     * PAUSE
     * LAND
     */
    std::string text = trim(raw);

    if (!text.empty() &&
        text.front() == '$') {
      text.erase(text.begin());
    }

    const auto checksum =
      text.find('*');

    if (checksum != std::string::npos) {
      text.erase(checksum);
    }

    if (text.rfind("CMD:", 0) == 0) {
      text.erase(0, 4);
    }

    std::stringstream stream(text);
    std::string token;

    if (std::getline(stream, token, ',')) {
      command_name =
        upper(trim(token));
    }

    while (std::getline(stream, token, ',')) {
      token = upper(trim(token));

      if (token.rfind("NOFLY=", 0) == 0) {
        token.erase(0, 6);
      }

      std::stringstream cell_stream(token);
      std::string cell;

      while (std::getline(
          cell_stream,
          cell,
          '|')) {
        cell = upper(trim(cell));

        if (!cell.empty()) {
          output.no_fly_cells.push_back(cell);
        }
      }
    }
  }

  for (auto& cell : output.no_fly_cells) {
    cell = upper(trim(cell));
  }

  if (command_name == "START") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_START;
  } else if (command_name == "PAUSE") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_PAUSE;
  } else if (command_name == "RESUME") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_RESUME;
  } else if (
    command_name == "RTL" ||
    command_name == "RETURN" ||
    command_name == "RETURN_HOME")
  {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_RTL;
  } else if (command_name == "LAND") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_LAND;
  } else if (command_name == "RESET") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_RESET;
  } else if (command_name == "CLEAR_TRACK") {
    output.cmd_type =
      ground_station_msgs::msg::GroundCommand::
      CMD_CLEAR_TRACK;
  }

  return output;
}

std::string UdpGroundStationBridgeNode::trim(
  const std::string& text)
{
  const auto first =
    std::find_if_not(
      text.begin(),
      text.end(),
      [](unsigned char c) {
        return std::isspace(c);
      });

  const auto last =
    std::find_if_not(
      text.rbegin(),
      text.rend(),
      [](unsigned char c) {
        return std::isspace(c);
      }).base();

  if (first >= last) {
    return "";
  }

  return std::string(first, last);
}

std::string UdpGroundStationBridgeNode::upper(
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

std::string UdpGroundStationBridgeNode::escape_json(
  const std::string& text)
{
  std::string result;
  result.reserve(text.size());

  for (const char c : text) {
    switch (c) {
      case '\\':
        result += "\\\\";
        break;

      case '"':
        result += "\\\"";
        break;

      case '\n':
        result += "\\n";
        break;

      case '\r':
        result += "\\r";
        break;

      default:
        result.push_back(c);
        break;
    }
  }

  return result;
}

std::string UdpGroundStationBridgeNode::json_string(
  const std::string& json,
  const std::string& key)
{
  const std::string key_text =
    "\"" + key + "\"";

  const auto key_pos =
    json.find(key_text);

  if (key_pos == std::string::npos) {
    return "";
  }

  const auto colon_pos =
    json.find(':', key_pos + key_text.size());

  if (colon_pos == std::string::npos) {
    return "";
  }

  const auto start =
    json.find('"', colon_pos + 1);

  if (start == std::string::npos) {
    return "";
  }

  const auto end =
    json.find('"', start + 1);

  if (end == std::string::npos) {
    return "";
  }

  return json.substr(
    start + 1,
    end - start - 1);
}

std::vector<std::string>
UdpGroundStationBridgeNode::json_string_array(
  const std::string& json,
  const std::string& key)
{
  std::vector<std::string> values;

  const std::string key_text =
    "\"" + key + "\"";

  const auto key_pos =
    json.find(key_text);

  if (key_pos == std::string::npos) {
    return values;
  }

  const auto array_start =
    json.find('[', key_pos + key_text.size());

  if (array_start == std::string::npos) {
    return values;
  }

  const auto array_end =
    json.find(']', array_start + 1);

  if (array_end == std::string::npos) {
    return values;
  }

  std::size_t position = array_start + 1;

  while (position < array_end) {
    const auto quote_start =
      json.find('"', position);

    if (quote_start == std::string::npos ||
        quote_start >= array_end) {
      break;
    }

    const auto quote_end =
      json.find('"', quote_start + 1);

    if (quote_end == std::string::npos ||
        quote_end > array_end) {
      break;
    }

    values.push_back(
      json.substr(
        quote_start + 1,
        quote_end - quote_start - 1));

    position = quote_end + 1;
  }

  return values;
}

}  // namespace ground_station_bridge_pkg