#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

#include "ground_station_msgs/msg/task_status.hpp"
#include "ground_station_msgs/msg/telemetry.hpp"
#include "ground_station_msgs/msg/ground_command.hpp"

#include "ground_station_bridge_pkg/telemetry_data.hpp"
#include "ground_station_bridge_pkg/serial_port.hpp"

namespace ground_station_bridge_pkg
{

class GroundStationBridgeNode : public rclcpp::Node
{
public:
  GroundStationBridgeNode();

private:
  void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void task_status_cb(const ground_station_msgs::msg::TaskStatus::SharedPtr msg);

  void raw_cmd_cb(const std_msgs::msg::String::SharedPtr msg);

  void tx_timer_cb();
  void rx_timer_cb();
  void telemetry_pub_timer_cb();

  float yaw_from_quat(float q0, float q1, float q2, float q3);
  ground_station_msgs::msg::GroundCommand parse_command(const std::string& raw) const;
  std::string trim_upper(std::string s) const;

private:
  TelemetryData telemetry_;
  SerialPort serial_;

  std::string serial_device_;
  int baudrate_;
  bool enable_serial_;
  bool print_tx_;

  std::string local_pos_topic_;
  std::string attitude_topic_;
  std::string vehicle_status_topic_;
  std::string task_status_topic_;
  std::string raw_cmd_topic_;
  std::string parsed_cmd_topic_;
  std::string telemetry_topic_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<ground_station_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_cmd_sub_;

  rclcpp::Publisher<ground_station_msgs::msg::GroundCommand>::SharedPtr parsed_cmd_pub_;
  rclcpp::Publisher<ground_station_msgs::msg::Telemetry>::SharedPtr telemetry_pub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_pub_timer_;
};

}  // namespace ground_station_bridge_pkg
