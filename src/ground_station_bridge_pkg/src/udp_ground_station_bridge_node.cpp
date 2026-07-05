#include "ground_station_bridge_pkg/udp_ground_station_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace ground_station_bridge_pkg
{

UdpGroundStationBridgeNode::UdpGroundStationBridgeNode()
: Node("udp_ground_station_bridge_node"),
  udp_fd_(-1)
{
  // ===== UDP 参数 =====
  declare_parameter<std::string>("udp_bind_ip", "0.0.0.0");
  declare_parameter<int>("udp_bind_port", 9000);

  declare_parameter<std::string>("udp_remote_ip", "192.168.43.50");
  declare_parameter<int>("udp_remote_port", 9001);

  declare_parameter<bool>("print_tx", true);
  declare_parameter<bool>("print_rx", true);

  // ===== ROS topic 参数 =====
  declare_parameter<std::string>("local_pos_topic", "/fmu/out/vehicle_local_position");
  declare_parameter<std::string>("attitude_topic", "/fmu/out/vehicle_attitude");
  declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
  declare_parameter<std::string>("task_status_topic", "/ground_station/task_status");
  declare_parameter<std::string>("raw_cmd_topic", "/ground_station/cmd");
  declare_parameter<std::string>("parsed_cmd_topic", "/ground_station/cmd_parsed");
  declare_parameter<std::string>("telemetry_topic", "/ground_station/telemetry");

  get_parameter("udp_bind_ip", udp_bind_ip_);
  get_parameter("udp_bind_port", udp_bind_port_);

  get_parameter("udp_remote_ip", udp_remote_ip_);
  get_parameter("udp_remote_port", udp_remote_port_);

  get_parameter("print_tx", print_tx_);
  get_parameter("print_rx", print_rx_);

  get_parameter("local_pos_topic", local_pos_topic_);
  get_parameter("attitude_topic", attitude_topic_);
  get_parameter("vehicle_status_topic", vehicle_status_topic_);
  get_parameter("task_status_topic", task_status_topic_);
  get_parameter("raw_cmd_topic", raw_cmd_topic_);
  get_parameter("parsed_cmd_topic", parsed_cmd_topic_);
  get_parameter("telemetry_topic", telemetry_topic_);

  // ===== UDP 初始化 =====
  if (!udp_init()) {
    RCLCPP_ERROR(get_logger(), "UDP init failed");
  }

  // ===== PX4 QoS =====
  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

  local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    local_pos_topic_, px4_qos,
    std::bind(&UdpGroundStationBridgeNode::local_pos_cb, this, std::placeholders::_1));

  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    attitude_topic_, px4_qos,
    std::bind(&UdpGroundStationBridgeNode::attitude_cb, this, std::placeholders::_1));

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, px4_qos,
    std::bind(&UdpGroundStationBridgeNode::status_cb, this, std::placeholders::_1));

  task_status_sub_ = create_subscription<ground_station_msgs::msg::TaskStatus>(
    task_status_topic_, 10,
    std::bind(&UdpGroundStationBridgeNode::task_status_cb, this, std::placeholders::_1));

  // 保留 ROS-only 调试入口：
  // ros2 topic pub /ground_station/cmd std_msgs/msg/String "{data: 'START'}" -1
  raw_cmd_sub_ = create_subscription<std_msgs::msg::String>(
    raw_cmd_topic_, 10,
    std::bind(&UdpGroundStationBridgeNode::raw_cmd_cb, this, std::placeholders::_1));

  parsed_cmd_pub_ = create_publisher<ground_station_msgs::msg::GroundCommand>(
    parsed_cmd_topic_, 10);

  telemetry_pub_ = create_publisher<ground_station_msgs::msg::Telemetry>(
    telemetry_topic_, 10);

  tx_timer_ = create_wall_timer(
    200ms, std::bind(&UdpGroundStationBridgeNode::tx_timer_cb, this));

  rx_timer_ = create_wall_timer(
    50ms, std::bind(&UdpGroundStationBridgeNode::rx_timer_cb, this));

  telemetry_pub_timer_ = create_wall_timer(
    200ms, std::bind(&UdpGroundStationBridgeNode::telemetry_pub_timer_cb, this));

  RCLCPP_INFO(get_logger(), "udp_ground_station_bridge_node started");
  RCLCPP_INFO(get_logger(), "UDP bind   : %s:%d", udp_bind_ip_.c_str(), udp_bind_port_);
  RCLCPP_INFO(get_logger(), "UDP remote : %s:%d", udp_remote_ip_.c_str(), udp_remote_port_);
  RCLCPP_INFO(get_logger(), "task_status_topic: %s", task_status_topic_.c_str());
  RCLCPP_INFO(get_logger(), "raw_cmd_topic    : %s", raw_cmd_topic_.c_str());
  RCLCPP_INFO(get_logger(), "parsed_cmd_topic : %s", parsed_cmd_topic_.c_str());
  RCLCPP_INFO(get_logger(), "telemetry_topic  : %s", telemetry_topic_.c_str());
}

UdpGroundStationBridgeNode::~UdpGroundStationBridgeNode()
{
  udp_close();
}

bool UdpGroundStationBridgeNode::udp_init()
{
  udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
    return false;
  }

  int reuse = 1;
  if (::setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
    RCLCPP_WARN(get_logger(), "setsockopt(SO_REUSEADDR) failed: %s", std::strerror(errno));
  }

  // 设置非阻塞，防止 rx_timer 卡死 ROS2
  int flags = ::fcntl(udp_fd_, F_GETFL, 0);
  if (flags >= 0) {
    ::fcntl(udp_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  std::memset(&local_addr_, 0, sizeof(local_addr_));
  local_addr_.sin_family = AF_INET;
  local_addr_.sin_port = htons(static_cast<uint16_t>(udp_bind_port_));

  if (::inet_pton(AF_INET, udp_bind_ip_.c_str(), &local_addr_.sin_addr) <= 0) {
    RCLCPP_ERROR(get_logger(), "invalid udp_bind_ip: %s", udp_bind_ip_.c_str());
    udp_close();
    return false;
  }

  if (::bind(udp_fd_, reinterpret_cast<struct sockaddr*>(&local_addr_), sizeof(local_addr_)) < 0) {
    RCLCPP_ERROR(
      get_logger(),
      "bind(%s:%d) failed: %s",
      udp_bind_ip_.c_str(),
      udp_bind_port_,
      std::strerror(errno));
    udp_close();
    return false;
  }

  std::memset(&remote_addr_, 0, sizeof(remote_addr_));
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_port = htons(static_cast<uint16_t>(udp_remote_port_));

  if (::inet_pton(AF_INET, udp_remote_ip_.c_str(), &remote_addr_.sin_addr) <= 0) {
    RCLCPP_ERROR(get_logger(), "invalid udp_remote_ip: %s", udp_remote_ip_.c_str());
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

void UdpGroundStationBridgeNode::local_pos_cb(
  const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  telemetry_.local_pos_valid = msg->xy_valid && msg->z_valid;
  telemetry_.x = msg->x;
  telemetry_.y = msg->y;
  telemetry_.z = msg->z;
  telemetry_.vx = msg->vx;
  telemetry_.vy = msg->vy;
  telemetry_.vz = msg->vz;
  telemetry_.stamp_us = msg->timestamp;
}

void UdpGroundStationBridgeNode::attitude_cb(
  const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  telemetry_.attitude_valid = true;
  telemetry_.yaw = yaw_from_quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  telemetry_.stamp_us = msg->timestamp;
}

void UdpGroundStationBridgeNode::status_cb(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  telemetry_.status_valid = true;
  telemetry_.arming_state = msg->arming_state;
  telemetry_.nav_state = msg->nav_state;
  telemetry_.stamp_us = msg->timestamp;
}

void UdpGroundStationBridgeNode::task_status_cb(
  const ground_station_msgs::msg::TaskStatus::SharedPtr msg)
{
  telemetry_.task_name = msg->task_name;
  telemetry_.current_wp = msg->current_wp;
  telemetry_.total_wp = msg->total_wp;
  telemetry_.mission_done = msg->mission_done;

  telemetry_.target_type = msg->target_type;
  telemetry_.target_name = msg->target_name;
}

float UdpGroundStationBridgeNode::yaw_from_quat(float q0, float q1, float q2, float q3)
{
  const float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  const float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string UdpGroundStationBridgeNode::trim_upper(std::string s) const
{
  s.erase(
    std::remove_if(
      s.begin(),
      s.end(),
      [](unsigned char c) {
        return std::isspace(c);
      }),
    s.end());

  std::transform(
    s.begin(),
    s.end(),
    s.begin(),
    [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });

  return s;
}

ground_station_msgs::msg::GroundCommand
UdpGroundStationBridgeNode::parse_command(const std::string& raw) const
{
  ground_station_msgs::msg::GroundCommand out;
  out.header.stamp = now();
  out.raw = raw;
  out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_UNKNOWN;

  // ===== GOTO 默认值 =====
  out.has_goal = false;
  out.x = 0.0f;
  out.y = 0.0f;
  out.z = 1.0f;

  std::string cmd = trim_upper(raw);

  // 去掉帧头 $
  if (!cmd.empty() && cmd.front() == '$') {
    cmd.erase(cmd.begin());
  }

  // 去掉校验部分 *XX
  auto star_pos = cmd.find('*');
  if (star_pos != std::string::npos) {
    cmd = cmd.substr(0, star_pos);
  }

  // 支持：
  // $CMD:START*XX
  // $CMD:GOTO,X=1.2,Y=-0.5,Z=1.0*XX
  // START
  // GOTO,X=1.2,Y=-0.5,Z=1.0
  if (cmd.rfind("CMD:", 0) == 0) {
    cmd = cmd.substr(4);
  } else if (cmd.rfind("CMD=", 0) == 0) {
    cmd = cmd.substr(4);
  } else if (cmd.rfind("TYPE=", 0) == 0) {
    cmd = cmd.substr(5);
  }

  // =========================
  // GOTO 坐标解析
  // 格式：
  // GOTO,X=1.20,Y=-0.50,Z=1.00
  // =========================
  if (cmd.rfind("GOTO", 0) == 0) {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_GOTO;
    out.has_goal = true;

    auto read_float = [&](const std::string& key, float& value) -> bool {
      const std::string tag = key + "=";
      const auto p = cmd.find(tag);
      if (p == std::string::npos) {
        return false;
      }

      const auto value_start = p + tag.size();
      auto value_end = cmd.find(",", value_start);
      if (value_end == std::string::npos) {
        value_end = cmd.size();
      }

      try {
        value = std::stof(cmd.substr(value_start, value_end - value_start));
        return true;
      } catch (...) {
        return false;
      }
    };

    read_float("X", out.x);
    read_float("Y", out.y);
    read_float("Z", out.z);

    RCLCPP_INFO(
      get_logger(),
      "Parsed GOTO: x=%.2f y=%.2f z=%.2f",
      out.x, out.y, out.z);

    return out;
  }

  // =========================
  // 普通命令解析
  // =========================
  if (cmd == "START") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_START;
  } else if (cmd == "PAUSE") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_PAUSE;
  } else if (cmd == "RESUME") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_RESUME;
  } else if (cmd == "RTL" || cmd == "RETURN" || cmd == "RETURN_HOME") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_RTL;
  } else if (cmd == "LAND") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_LAND;
  } else if (cmd == "RESET") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_RESET;
  } else if (cmd == "CLEAR_TRACK") {
    out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_CLEAR_TRACK;
  }

  return out;
}

void UdpGroundStationBridgeNode::raw_cmd_cb(const std_msgs::msg::String::SharedPtr msg)
{
  auto parsed = parse_command(msg->data);
  parsed_cmd_pub_->publish(parsed);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "CMD raw='%s' parsed=%u",
    msg->data.c_str(),
    static_cast<unsigned>(parsed.cmd_type));
}

void UdpGroundStationBridgeNode::telemetry_pub_timer_cb()
{
  ground_station_msgs::msg::Telemetry msg;
  msg.header.stamp = now();

  msg.local_pos_valid = telemetry_.local_pos_valid;
  msg.attitude_valid = telemetry_.attitude_valid;
  msg.status_valid = telemetry_.status_valid;

  msg.x = telemetry_.x;
  msg.y = telemetry_.y;
  msg.z = telemetry_.z;

  msg.vx = telemetry_.vx;
  msg.vy = telemetry_.vy;
  msg.vz = telemetry_.vz;

  msg.yaw = telemetry_.yaw;

  msg.arming_state = telemetry_.arming_state;
  msg.nav_state = telemetry_.nav_state;

  msg.task_name = telemetry_.task_name;
  msg.current_wp = telemetry_.current_wp;
  msg.total_wp = telemetry_.total_wp;
  msg.mission_done = telemetry_.mission_done;
  msg.target_type = telemetry_.target_type;
  msg.target_name = telemetry_.target_name;

  telemetry_pub_->publish(msg);
}

void UdpGroundStationBridgeNode::tx_timer_cb()
{
  const std::string frame = Protocol::encode_status(telemetry_);

  if (print_tx_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "UDP TX %s:%d : %s",
      udp_remote_ip_.c_str(),
      udp_remote_port_,
      frame.c_str());
  }

  if (udp_fd_ < 0) {
    return;
  }

  const ssize_t sent = ::sendto(
    udp_fd_,
    frame.data(),
    frame.size(),
    0,
    reinterpret_cast<struct sockaddr*>(&remote_addr_),
    sizeof(remote_addr_));

  if (sent < 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "UDP sendto failed: %s",
      std::strerror(errno));
  }
}

void UdpGroundStationBridgeNode::rx_timer_cb()
{
  if (udp_fd_ < 0) {
    return;
  }

  char buffer[1024] = {0};

  while (true) {
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    std::memset(&src_addr, 0, sizeof(src_addr));

    const ssize_t n = ::recvfrom(
      udp_fd_,
      buffer,
      sizeof(buffer) - 1,
      0,
      reinterpret_cast<struct sockaddr*>(&src_addr),
      &src_len);

    if (n <= 0) {
      break;
    }

    std::string rx(buffer, buffer + n);

    if (print_rx_) {
      char src_ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &src_addr.sin_addr, src_ip, sizeof(src_ip));
      const int src_port = ntohs(src_addr.sin_port);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 200,
        "UDP RX %s:%d : %s",
        src_ip,
        src_port,
        rx.c_str());
    }

    std_msgs::msg::String raw_msg;
    raw_msg.data = rx;
    raw_cmd_cb(std::make_shared<std_msgs::msg::String>(raw_msg));
  }
}

}  // namespace ground_station_bridge_pkg