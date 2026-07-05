#include "ground_station_bridge_pkg/ground_station_bridge_node.hpp"
#include "ground_station_bridge_pkg/protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>

using namespace std::chrono_literals;

namespace ground_station_bridge_pkg
{

GroundStationBridgeNode::GroundStationBridgeNode()
: Node("ground_station_bridge_node")
{
  declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<bool>("enable_serial", false);
  declare_parameter<bool>("print_tx", true);

  declare_parameter<std::string>("local_pos_topic", "/fmu/out/vehicle_local_position");
  declare_parameter<std::string>("attitude_topic", "/fmu/out/vehicle_attitude");
  declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status_v1");
  declare_parameter<std::string>("task_status_topic", "/ground_station/task_status");
  declare_parameter<std::string>("raw_cmd_topic", "/ground_station/cmd");
  declare_parameter<std::string>("parsed_cmd_topic", "/ground_station/cmd_parsed");
  declare_parameter<std::string>("telemetry_topic", "/ground_station/telemetry");

  get_parameter("serial_device", serial_device_);
  get_parameter("baudrate", baudrate_);
  get_parameter("enable_serial", enable_serial_);
  get_parameter("print_tx", print_tx_);

  get_parameter("local_pos_topic", local_pos_topic_);
  get_parameter("attitude_topic", attitude_topic_);
  get_parameter("vehicle_status_topic", vehicle_status_topic_);
  get_parameter("task_status_topic", task_status_topic_);
  get_parameter("raw_cmd_topic", raw_cmd_topic_);
  get_parameter("parsed_cmd_topic", parsed_cmd_topic_);
  get_parameter("telemetry_topic", telemetry_topic_);

  auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

  local_pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    local_pos_topic_, px4_qos,
    std::bind(&GroundStationBridgeNode::local_pos_cb, this, std::placeholders::_1));

  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
    attitude_topic_, px4_qos,
    std::bind(&GroundStationBridgeNode::attitude_cb, this, std::placeholders::_1));

  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, px4_qos,
    std::bind(&GroundStationBridgeNode::status_cb, this, std::placeholders::_1));

  task_status_sub_ = create_subscription<ground_station_msgs::msg::TaskStatus>(
    task_status_topic_, 10,
    std::bind(&GroundStationBridgeNode::task_status_cb, this, std::placeholders::_1));

  raw_cmd_sub_ = create_subscription<std_msgs::msg::String>(
    raw_cmd_topic_, 10,
    std::bind(&GroundStationBridgeNode::raw_cmd_cb, this, std::placeholders::_1));

  parsed_cmd_pub_ = create_publisher<ground_station_msgs::msg::GroundCommand>(
    parsed_cmd_topic_, 10);

  telemetry_pub_ = create_publisher<ground_station_msgs::msg::Telemetry>(
    telemetry_topic_, 10);

  if (enable_serial_) {
    if (serial_.open_port(serial_device_, baudrate_)) {
      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d",
                  serial_device_.c_str(), baudrate_);
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to open serial: %s @ %d",
                   serial_device_.c_str(), baudrate_);
    }
  } else {
    RCLCPP_WARN(get_logger(), "Serial disabled. Running in ROS-only debug mode.");
  }

  tx_timer_ = create_wall_timer(
    200ms, std::bind(&GroundStationBridgeNode::tx_timer_cb, this));

  rx_timer_ = create_wall_timer(
    50ms, std::bind(&GroundStationBridgeNode::rx_timer_cb, this));

  telemetry_pub_timer_ = create_wall_timer(
    200ms, std::bind(&GroundStationBridgeNode::telemetry_pub_timer_cb, this));

  RCLCPP_INFO(get_logger(), "ground_station_bridge_node started");
  RCLCPP_INFO(get_logger(), "task_status_topic: %s", task_status_topic_.c_str());
  RCLCPP_INFO(get_logger(), "raw_cmd_topic    : %s", raw_cmd_topic_.c_str());
  RCLCPP_INFO(get_logger(), "parsed_cmd_topic : %s", parsed_cmd_topic_.c_str());
  RCLCPP_INFO(get_logger(), "telemetry_topic  : %s", telemetry_topic_.c_str());
}

void GroundStationBridgeNode::local_pos_cb(
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

void GroundStationBridgeNode::attitude_cb(
  const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  telemetry_.attitude_valid = true;
  telemetry_.yaw = yaw_from_quat(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  telemetry_.stamp_us = msg->timestamp;
}

void GroundStationBridgeNode::status_cb(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  telemetry_.status_valid = true;
  telemetry_.arming_state = msg->arming_state;
  telemetry_.nav_state = msg->nav_state;
  telemetry_.stamp_us = msg->timestamp;
}

void GroundStationBridgeNode::task_status_cb(
  const ground_station_msgs::msg::TaskStatus::SharedPtr msg)
{
  telemetry_.task_name = msg->task_name;
  telemetry_.current_wp = msg->current_wp;
  telemetry_.total_wp = msg->total_wp;
  telemetry_.mission_done = msg->mission_done;
}

float GroundStationBridgeNode::yaw_from_quat(float q0, float q1, float q2, float q3)
{
  const float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  const float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string GroundStationBridgeNode::trim_upper(std::string s) const
{
  s.erase(std::remove_if(s.begin(), s.end(),
           [](unsigned char c) { return std::isspace(c); }),
           s.end());

  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

ground_station_msgs::msg::GroundCommand
GroundStationBridgeNode::parse_command(const std::string& raw) const
{
  ground_station_msgs::msg::GroundCommand out;
  out.header.stamp = now();
  out.raw = raw;
  out.cmd_type = ground_station_msgs::msg::GroundCommand::CMD_UNKNOWN;

  std::string cmd = trim_upper(raw);

  if (!cmd.empty() && cmd.front() == '$') {
    cmd.erase(cmd.begin());
  }

  auto star_pos = cmd.find('*');
  if (star_pos != std::string::npos) {
    cmd = cmd.substr(0, star_pos);
  }

  if (cmd.rfind("CMD:", 0) == 0) {
    cmd = cmd.substr(4);
  } else if (cmd.rfind("CMD=", 0) == 0) {
    cmd = cmd.substr(4);
  } else if (cmd.rfind("TYPE=", 0) == 0) {
    cmd = cmd.substr(5);
  }

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

void GroundStationBridgeNode::raw_cmd_cb(const std_msgs::msg::String::SharedPtr msg)
{
  auto parsed = parse_command(msg->data);
  parsed_cmd_pub_->publish(parsed);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "CMD raw='%s' parsed=%u",
    msg->data.c_str(),
    static_cast<unsigned>(parsed.cmd_type));
}

void GroundStationBridgeNode::telemetry_pub_timer_cb()
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

  telemetry_pub_->publish(msg);
}

void GroundStationBridgeNode::tx_timer_cb()
{
  const std::string frame = Protocol::encode_status(telemetry_);

  if (print_tx_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "TX: %s", frame.c_str());
  }

  if (enable_serial_ && serial_.is_open()) {
    if (!serial_.write_data(frame)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Serial write failed");
    }
  }
}

void GroundStationBridgeNode::rx_timer_cb()
{
  if (!enable_serial_ || !serial_.is_open()) {
    return;
  }

  char buffer[256] = {0};
  const int n = serial_.read_data(buffer, sizeof(buffer) - 1);
  if (n <= 0) {
    return;
  }

  std::string rx(buffer, buffer + n);

  std_msgs::msg::String raw_msg;
  raw_msg.data = rx;
  raw_cmd_cb(std::make_shared<std_msgs::msg::String>(raw_msg));
}

}  // namespace ground_station_bridge_pkg
